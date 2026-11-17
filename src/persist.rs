use std::ffi::CString;

use esp_idf_matter::error::to_persist_error;
use esp_idf_matter::matter::error::Error;
use esp_idf_matter::matter::persist::KvBlobStore;
use esp_idf_svc::handle::RawHandle;
use esp_idf_svc::nvs::{EspDefaultNvs, EspDefaultNvsPartition};
use esp_idf_svc::sys::{nvs_commit, nvs_set_blob, EspError};

// Same namespace and key format as upstream's `EspKvBlobStore`.
pub struct MatterBlobStore(EspDefaultNvs);

impl MatterBlobStore {
    pub fn new(nvs: EspDefaultNvsPartition) -> Result<Self, EspError> {
        Ok(Self(EspDefaultNvs::new(nvs, "esp-idf-matter", true)?))
    }
}

impl KvBlobStore for MatterBlobStore {
    fn load<'a>(&mut self, key: u16, buf: &'a mut [u8]) -> Result<Option<&'a [u8]>, Error> {
        self.0
            .get_blob(&format!("{key:04x}"), buf)
            .map_err(to_persist_error)
    }

    fn store(&mut self, key: u16, data: &[u8], _buf: &mut [u8]) -> Result<(), Error> {
        let handle = self.0.handle();
        let key = CString::new(format!("{key:04x}")).expect("hex key contains no NUL");

        // Bypass the delete-first EspNvs wrapper. The handle, C string and data
        // stay valid throughout both synchronous NVS calls.
        checked(unsafe { nvs_set_blob(handle, key.as_ptr(), data.as_ptr().cast(), data.len()) })?;
        checked(unsafe { nvs_commit(handle) })
    }

    fn remove(&mut self, key: u16, _buf: &mut [u8]) -> Result<(), Error> {
        self.0
            .remove(&format!("{key:04x}"))
            .map(|_| ())
            .map_err(to_persist_error)
    }
}

fn checked(code: i32) -> Result<(), Error> {
    EspError::from(code).map_or(Ok(()), |err| Err(to_persist_error(err)))
}
