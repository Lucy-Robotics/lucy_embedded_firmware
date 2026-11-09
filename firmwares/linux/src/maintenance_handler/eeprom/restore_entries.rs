//! Converting restore-homing entries into the records the bus write actually needs.

use lucy_embedded_firmware_feetech::feetech::HomingRecord;

use crate::config::FEETECH_0;
use crate::maintenance::RestoreEntry;

/// Exactly one valid entry per configured servo, in configuration order.
pub(super) fn records_from_entries(entries: &[RestoreEntry]) -> Option<[HomingRecord; 6]> {
    if entries.len() != FEETECH_0.servos.len() {
        return None;
    }
    let mut records = [HomingRecord::default(); 6];
    for (record, servo) in records.iter_mut().zip(FEETECH_0.servos.iter()) {
        let entry = entries.iter().find(|e| e.id == servo.id)?;
        *record = homing_record_from_entry(entry);
        if !record.is_valid() {
            return None;
        }
    }
    Some(records)
}

fn homing_record_from_entry(entry: &RestoreEntry) -> HomingRecord {
    HomingRecord {
        homing_offset: entry.homing_offset,
        min_limit: entry.min_limit,
        max_limit: entry.max_limit,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::entries;

    #[test]
    fn homing_record_from_entry_copies_every_field() {
        let entry = RestoreEntry {
            id: 1,
            homing_offset: -5,
            min_limit: 10,
            max_limit: 4000,
        };
        let record = homing_record_from_entry(&entry);
        assert_eq!(record.homing_offset, -5);
        assert_eq!(record.min_limit, 10);
        assert_eq!(record.max_limit, 4000);
    }

    #[test]
    fn restore_needs_every_servo_exactly_once_with_valid_values() {
        assert!(records_from_entries(&entries()).is_some());

        let mut missing = entries();
        missing.pop();
        assert!(records_from_entries(&missing).is_none());

        let mut wrong_id = entries();
        wrong_id[5].id = 9;
        assert!(records_from_entries(&wrong_id).is_none());

        let mut too_large = entries();
        too_large[0].max_limit = 5000;
        assert!(records_from_entries(&too_large).is_none());

        let mut too_far = entries();
        too_far[0].homing_offset = 2048;
        assert!(records_from_entries(&too_far).is_none());
    }
}
