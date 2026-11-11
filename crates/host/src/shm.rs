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

/// POSIX shm name of a ros2_control node: `/<node>.domain<ROS_DOMAIN_ID>`.
///
/// The domain keeps robots on separate ROS domains of one host on separate segments.
/// ROS_DOMAIN_ID unset or not a plain number counts as 0. lucy_ros2_control creates the
/// segment under the same name (`lucy_shm_name` in shared_memory_channel.cpp): keep both
/// in step.
pub fn shm_name(node: &str) -> String {
    shm_name_in_domain(node, std::env::var("ROS_DOMAIN_ID").ok().as_deref())
}

fn shm_name_in_domain(node: &str, domain: Option<&str>) -> String {
    let domain = domain
        .filter(|d| !d.is_empty() && d.bytes().all(|b| b.is_ascii_digit()))
        .and_then(|d| d.parse::<u32>().ok())
        .unwrap_or(0);
    format!("/{}.domain{domain}", node.trim_start_matches('/'))
}

pub struct Shm {
    table: SharedTable,
    _map: MmapMut,
}

impl Shm {
    pub fn attach(node: &str) -> Self {
        let name = shm_name(node);
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
                        eprintln!(
                            "Waiting for shared memory {name} from the ROS 2 hardware interface: {error}"
                        );
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
    let table =
        unsafe { SharedTable::from_raw(map.as_mut_ptr(), map.len()) }.map_err(str::to_string)?;

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

#[cfg(test)]
mod tests {
    use super::shm_name_in_domain;

    // lucy_ros2_control's test_shared_memory_channel.cpp (LucyShmName) checks the same cases.
    #[test]
    fn name_carries_the_ros_domain() {
        assert_eq!(shm_name_in_domain("so_arm", Some("3")), "/so_arm.domain3");
        assert_eq!(shm_name_in_domain("/so_arm", Some("3")), "/so_arm.domain3");
        assert_eq!(shm_name_in_domain("so_arm", Some("07")), "/so_arm.domain7");
    }

    #[test]
    fn missing_or_invalid_domain_is_zero() {
        for domain in [None, Some(""), Some("abc"), Some("3x"), Some("-1"), Some("+3")] {
            assert_eq!(shm_name_in_domain("so_arm", domain), "/so_arm.domain0", "{domain:?}");
        }
    }
}
