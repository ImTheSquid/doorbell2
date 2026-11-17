use std::ffi::CString;

use esp_idf_matter::error::to_persist_error;
use esp_idf_matter::matter::error::Error;
use esp_idf_matter::matter::persist::KvBlobStore;
use esp_idf_svc::handle::RawHandle;
use esp_idf_svc::nvs::{EspDefaultNvs, EspDefaultNvsPartition};
use esp_idf_svc::sys::{nvs_commit, nvs_set_blob, EspError};
use log::warn;

// Same namespace and key format as upstream's `EspKvBlobStore`.
pub struct MatterBlobStore(EspDefaultNvs);

impl MatterBlobStore {
    pub fn new(nvs: EspDefaultNvsPartition) -> Result<Self, EspError> {
        Ok(Self(EspDefaultNvs::new(nvs, "esp-idf-matter", true)?))
    }

    /// The previous stack kept all fabrics under key 0, basic info under 1 and
    /// networks under 2, in a format this one cannot decode. Key 0 is unused
    /// now, so it marks that layout. It is erased last so an interrupted wipe
    /// is retried on the next boot.
    pub fn erase_legacy_layout(&mut self) -> Result<(), EspError> {
        if self.0.blob_len("0000")?.is_some() {
            warn!("Erasing state from the previous Matter stack; re-pair the device");
            for key in ["0001", "0002", "0000"] {
                self.0.remove(key)?;
            }
        }
        Ok(())
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
