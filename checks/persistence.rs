//! Host regression tests for the production MatterBlobStore with a mocked NVS API.
//! Run: rustc --edition=2021 --test checks/persistence.rs -o /tmp/doorbell2-persist-tests
//! This checks adapter behavior, not physical flash recovery.
#![allow(async_fn_in_trait)]

extern crate self as esp_idf_matter;
extern crate self as esp_idf_svc;

use std::{
    cell::RefCell,
    collections::BTreeMap,
    future::Future,
    task::{Context, Poll, Waker},
};

thread_local! { static FLASH: RefCell<Flash> = RefCell::new(Flash::default()); }
#[derive(Default)]
struct Flash {
    blobs: BTreeMap<String, Vec<u8>>,
    calls: Vec<&'static str>,
    fail_write: bool,
    fail_commit: bool,
    cut_before_write: bool,
}

pub mod sys {
    use super::FLASH;
    use std::ffi::{c_char, c_void, CStr};

    #[derive(Debug)]
    pub struct EspError(pub i32);
    impl EspError {
        pub fn from(code: i32) -> Option<Self> {
            (code != 0).then_some(Self(code))
        }
    }

    pub unsafe fn nvs_set_blob(_: u32, key: *const c_char, data: *const c_void, len: usize) -> i32 {
        let key = unsafe { CStr::from_ptr(key) }.to_str().unwrap();
        let data = unsafe { std::slice::from_raw_parts(data.cast::<u8>(), len) };
        FLASH.with(|flash| {
            let mut flash = flash.borrow_mut();
            flash.calls.push("set");
            if flash.cut_before_write {
                panic!("simulated interruption before NVS replacement");
            }
            if flash.fail_write {
                return 1;
            }
            flash.blobs.insert(key.to_owned(), data.to_vec());
            0
        })
    }

    pub unsafe fn nvs_commit(_: u32) -> i32 {
        FLASH.with(|flash| {
            let mut flash = flash.borrow_mut();
            flash.calls.push("commit");
            i32::from(flash.fail_commit)
        })
    }
}

pub mod handle {
    pub trait RawHandle {
        type Handle;
        fn handle(&self) -> Self::Handle;
    }
}

pub mod nvs {
    use super::{handle::RawHandle, sys::EspError, FLASH};
    #[derive(Clone)]
    pub struct EspDefaultNvsPartition;
    pub struct EspDefaultNvs;
    impl EspDefaultNvs {
        pub fn new(
            _: EspDefaultNvsPartition,
            namespace: &str,
            read_write: bool,
        ) -> Result<Self, EspError> {
            assert_eq!(
                namespace, "esp-idf-matter",
                "must load previously commissioned devices"
            );
            assert!(read_write);
            Ok(Self)
        }
        pub fn get_blob<'a>(
            &self,
            key: &str,
            buf: &'a mut [u8],
        ) -> Result<Option<&'a [u8]>, EspError> {
            FLASH.with(|flash| {
                let flash = flash.borrow();
                let Some(blob) = flash.blobs.get(key) else {
                    return Ok(None);
                };
                if blob.len() > buf.len() {
                    return Err(EspError::from(2).unwrap());
                }
                buf[..blob.len()].copy_from_slice(blob);
                Ok(Some(&buf[..blob.len()]))
            })
        }
        pub fn remove(&self, key: &str) -> Result<bool, EspError> {
            FLASH.with(|flash| {
                let mut flash = flash.borrow_mut();
                flash.calls.push("remove");
                Ok(flash.blobs.remove(key).is_some())
            })
        }
        pub fn set_blob(&self, _: &str, _: &[u8]) -> Result<(), EspError> {
            panic!("the production adapter must bypass the pre-erasing wrapper");
        }
    }
    impl RawHandle for EspDefaultNvs {
        type Handle = u32;
        fn handle(&self) -> u32 {
            1
        }
    }
}

pub mod matter {
    pub mod error {
        #[derive(Debug, PartialEq)]
        pub struct Error(pub i32);
    }
}
pub mod error {
    pub fn to_persist_error(error: crate::sys::EspError) -> crate::matter::error::Error {
        crate::matter::error::Error(error.0)
    }
}
pub mod stack {
    pub mod persist {
        use crate::matter::error::Error;
        // The callback-based contract used by the application's SDK API.
        pub trait KvBlobStore {
            async fn load<F>(&mut self, key: u16, buf: &mut [u8], cb: F) -> Result<(), Error>
            where
                F: FnOnce(Option<&[u8]>) -> Result<(), Error>;
            async fn store<F>(&mut self, key: u16, buf: &mut [u8], cb: F) -> Result<(), Error>
            where
                F: FnOnce(&mut [u8]) -> Result<usize, Error>;
            async fn remove(&mut self, key: u16, buf: &mut [u8]) -> Result<(), Error>;
        }
    }
}

#[path = "../src/persist.rs"]
mod production;
use production::MatterBlobStore;
use stack::persist::KvBlobStore;

// These adapter operations are synchronous NVS calls inside async trait methods.
fn run<F: Future>(future: F) -> F::Output {
    let mut future = std::pin::pin!(future);
    match future
        .as_mut()
        .poll(&mut Context::from_waker(Waker::noop()))
    {
        Poll::Ready(value) => value,
        Poll::Pending => panic!("NVS adapter unexpectedly waited"),
    }
}
fn setup() -> MatterBlobStore {
    FLASH.with(|flash| {
        *flash.borrow_mut() = Flash {
            blobs: [
                ("0000".into(), b"old-fabrics".to_vec()),
                ("0002".into(), b"wifi".to_vec()),
            ]
            .into(),
            ..Flash::default()
        }
    });
    MatterBlobStore::new(nvs::EspDefaultNvsPartition).unwrap()
}
fn write(store: &mut MatterBlobStore, key: u16) -> Result<(), matter::error::Error> {
    run(store.store(key, &mut [0; 64], |buf| {
        buf[..3].copy_from_slice(b"new");
        Ok(3)
    }))
}
fn read(store: &mut MatterBlobStore, key: u16) -> Option<Vec<u8>> {
    let mut result = None;
    run(store.load(key, &mut [0; 64], |data| {
        result = data.map(<[u8]>::to_vec);
        Ok(())
    }))
    .unwrap();
    result
}

#[test]
fn existing_namespace_and_keys_load_without_migration() {
    let mut store = setup();
    assert_eq!(read(&mut store, 0), Some(b"old-fabrics".to_vec()));
    assert_eq!(read(&mut store, 2), Some(b"wifi".to_vec()));
    assert_eq!(read(&mut store, 3), None);
}
#[test]
fn replacement_commits_and_preserves_other_keys() {
    let mut store = setup();
    write(&mut store, 0).unwrap();
    assert_eq!(read(&mut store, 0), Some(b"new".to_vec()));
    assert_eq!(read(&mut store, 2), Some(b"wifi".to_vec()));
    FLASH.with(|f| assert_eq!(f.borrow().calls, ["set", "commit"]));
}
#[test]
fn failed_replacement_retains_the_old_blob() {
    let mut store = setup();
    FLASH.with(|f| f.borrow_mut().fail_write = true);
    assert!(write(&mut store, 0).is_err());
    assert_eq!(read(&mut store, 0), Some(b"old-fabrics".to_vec()));
    FLASH.with(|f| assert_eq!(f.borrow().calls, ["set"]));
}
#[test]
fn interrupted_replacement_retains_the_old_blob() {
    let mut store = setup();
    FLASH.with(|f| f.borrow_mut().cut_before_write = true);
    assert!(
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| write(&mut store, 0))).is_err()
    );
    assert_eq!(read(&mut store, 0), Some(b"old-fabrics".to_vec()));
}
#[test]
fn serialization_error_does_not_touch_storage() {
    let mut store = setup();
    assert_eq!(
        run(store.store(0, &mut [0; 64], |_| Err(matter::error::Error(9)))),
        Err(matter::error::Error(9))
    );
    assert_eq!(read(&mut store, 0), Some(b"old-fabrics".to_vec()));
    FLASH.with(|f| assert!(f.borrow().calls.is_empty()));
}
#[test]
fn commit_error_is_reported() {
    let mut store = setup();
    FLASH.with(|f| f.borrow_mut().fail_commit = true);
    assert!(write(&mut store, 0).is_err());
    FLASH.with(|f| assert_eq!(f.borrow().calls, ["set", "commit"]));
}
#[test]
fn explicit_remove_only_deletes_the_requested_key() {
    let mut store = setup();
    run(store.remove(0, &mut [])).unwrap();
    assert_eq!(read(&mut store, 0), None);
    assert_eq!(read(&mut store, 2), Some(b"wifi".to_vec()));
}
#[test]
fn hex_keys_retain_leading_zeroes() {
    let mut store = setup();
    write(&mut store, 0x000a).unwrap();
    FLASH.with(|f| assert!(f.borrow().blobs.contains_key("000a")));
    assert_eq!(read(&mut store, 0x000a), Some(b"new".to_vec()));
}
