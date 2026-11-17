use std::ffi::CString;

use esp_idf_matter::error::to_persist_error;
use esp_idf_matter::matter::error::Error;
use esp_idf_matter::stack::persist::KvBlobStore;
use esp_idf_svc::handle::RawHandle;
use esp_idf_svc::nvs::{EspDefaultNvs, EspDefaultNvsPartition};
use esp_idf_svc::sys::{nvs_commit, nvs_set_blob, EspError};

// Preserve the upstream namespace, keys and blob format for existing pairings.
pub struct MatterBlobStore(EspDefaultNvs);

impl MatterBlobStore {
    pub fn new(nvs: EspDefaultNvsPartition) -> Result<Self, EspError> {
        Ok(Self(EspDefaultNvs::new(nvs, "esp-idf-matter", true)?))
    }
}

impl KvBlobStore for MatterBlobStore {
    async fn load<F>(&mut self, key: u16, buf: &mut [u8], cb: F) -> Result<(), Error>
    where
        F: FnOnce(Option<&[u8]>) -> Result<(), Error>,
    {
        let key = format!("{key:04x}");
        cb(self.0.get_blob(&key, buf).map_err(to_persist_error)?)
    }

    async fn store<F>(&mut self, key: u16, buf: &mut [u8], cb: F) -> Result<(), Error>
    where
        F: FnOnce(&mut [u8]) -> Result<usize, Error>,
    {
        let len = cb(buf)?;
        let handle = self.0.handle();
        let key = CString::new(format!("{key:04x}")).expect("hex key contains no NUL");

        // Bypass the delete-first EspNvs wrapper. The handle, C string and buffer
        // stay valid throughout both synchronous NVS calls.
        checked(unsafe { nvs_set_blob(handle, key.as_ptr(), buf[..len].as_ptr().cast(), len) })?;
        checked(unsafe { nvs_commit(handle) })
    }

    async fn remove(&mut self, key: u16, _buf: &mut [u8]) -> Result<(), Error> {
        self.0
            .remove(&format!("{key:04x}"))
            .map(|_| ())
            .map_err(to_persist_error)
    }
}

fn checked(code: i32) -> Result<(), Error> {
    EspError::from(code).map_or(Ok(()), |err| Err(to_persist_error(err)))
}
