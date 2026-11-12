use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const MIN_INTERVAL: Duration = Duration::from_secs(1);

pub struct LinkFile {
    path: PathBuf,
    last_write: Option<Instant>,
}

impl LinkFile {
    pub fn for_node(node: &str) -> Self {
        Self::at(PathBuf::from(format!("/tmp/.lucy_mcu_bridge_link.{node}")))
    }

    pub fn at(path: PathBuf) -> Self {
        Self {
            path,
            last_write: None,
        }
    }

    pub fn record(&mut self) {
        let now = Instant::now();
        if self
            .last_write
            .is_some_and(|last| now - last < MIN_INTERVAL)
        {
            return;
        }
        let unix = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default();
        match std::fs::write(&self.path, format!("{:.3}\n", unix.as_secs_f64())) {
            Ok(()) => self.last_write = Some(now),
            Err(err) => eprintln!(
                "Could not record the serial link in {}: {err}",
                self.path.display()
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stamp(path: &PathBuf) -> f64 {
        std::fs::read_to_string(path)
            .unwrap()
            .trim()
            .parse()
            .unwrap()
    }

    #[test]
    fn records_the_unix_time_then_throttles() {
        let path = std::env::temp_dir().join(format!("lucy_link_test_{}", std::process::id()));
        let _ = std::fs::remove_file(&path);
        let before = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();

        let mut link = LinkFile::at(path.clone());
        link.record();
        let first = stamp(&path);
        assert!(first >= before - 0.01, "{first} < {before}");

        std::fs::remove_file(&path).unwrap();
        link.record();
        assert!(!path.exists());
    }

    #[test]
    fn node_file_is_the_one_the_flash_pipeline_reads() {
        assert_eq!(
            LinkFile::for_node("left_arm").path,
            PathBuf::from("/tmp/.lucy_mcu_bridge_link.left_arm")
        );
    }

    #[test]
    fn failed_write_is_retried_on_the_next_record() {
        let dir = std::env::temp_dir().join(format!("lucy_link_retry_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let path = dir.join("link");
        let mut link = LinkFile::at(path.clone());

        link.record();
        assert!(!path.exists());

        std::fs::create_dir(&dir).unwrap();
        link.record();
        assert!(path.exists());
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
