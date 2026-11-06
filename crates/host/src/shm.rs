use std::ffi::CString;
use std::fs::File;
use std::ops::{Deref, DerefMut};
use std::os::fd::FromRawFd;
use std::time::Duration;

use memmap2::MmapMut;

use lucy_embedded_firmware_core::data::{SharedTable, ShmHeader};

const RETRY_PERIOD: Duration = Duration::from_millis(200);

pub fn node_name() -> String {
    std::env::args()
        .nth(1)
        .or_else(|| std::env::var("LUCY_NODE_NAME").ok())
        .filter(|n| !n.is_empty())
        .unwrap_or_else(|| "lucy".to_string())
}

pub struct Shm {
    table: SharedTable,
    _map: MmapMut,
}

impl Shm {
    pub fn attach(node: &str) -> Self {
        let name = format!("/{node}");
        let c_name = CString::new(name.as_str()).expect("shm name contains a NUL byte");
        let mut last_error = String::new();

        loop {
            match try_attach(&name, &c_name) {
                Ok(shm) => {
                    println!("Attached to shared memory {name}");
                    return shm;
                }
                Err(error) => {
                    if error != last_error {
                        eprintln!("Waiting for shared memory {name} from the ROS 2 hardware interface: {error}");
                        last_error = error;
                    }
                    std::thread::sleep(RETRY_PERIOD);
                }
            }
        }
    }
}

fn try_attach(name: &str, c_name: &CString) -> Result<Shm, String> {
    let fd = unsafe { libc::shm_open(c_name.as_ptr(), libc::O_RDWR, 0) };
    if fd == -1 {
        return Err(std::io::Error::last_os_error().to_string());
    }
    let file = unsafe { File::from_raw_fd(fd) };

    let len = file.metadata().map_err(|e| e.to_string())?.len() as usize;
    if len < size_of::<ShmHeader>() {
        return Err(format!("{name} is {len} bytes, not initialized yet"));
    }

    let mut map = unsafe { MmapMut::map_mut(&file) }.map_err(|e| e.to_string())?;
    let table = unsafe { SharedTable::from_raw(map.as_mut_ptr(), map.len()) }.map_err(str::to_string)?;

    Ok(Shm { table, _map: map })
}

impl Deref for Shm {
    type Target = SharedTable;

    fn deref(&self) -> &SharedTable {
        &self.table
    }
}

impl DerefMut for Shm {
    fn deref_mut(&mut self) -> &mut SharedTable {
        &mut self.table
    }
}
