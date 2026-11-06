use std::io::Write;

use crate::usb_port::{Adapter, AdapterChoice, UsbPort, UsbPortConfig, choose_adapter};

/// Why the bus is, or is not, open.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Link {
    Open,
    NoAdapter,
    Ambiguous,
    SelectedMissing,
    OpenFailed,
}

impl Link {
    pub fn as_str(self) -> &'static str {
        match self {
            Link::Open => "open",
            Link::NoAdapter => "no_adapter",
            Link::Ambiguous => "ambiguous",
            Link::SelectedMissing => "selected_missing",
            Link::OpenFailed => "open_failed",
        }
    }
}

pub struct Resources {
    pub config: UsbPortConfig,
    pub usb0: Option<UsbPort>,
    pub open_adapter: Option<Adapter>,
    pub adapters: Vec<Adapter>,
    pub selected: Option<String>,
    pub link: Link,
    pub open_error: Option<String>,
}

impl Resources {
    pub fn new(config: UsbPortConfig) -> Self {
        Self {
            config,
            usb0: None,
            open_adapter: None,
            adapters: Vec::new(),
            selected: None,
            link: Link::NoAdapter,
            open_error: None,
        }
    }

    /// Follow plugged and unplugged adapters: close a port that is gone, open the chosen one.
    pub fn refresh(&mut self) {
        self.adapters = self.config.list_adapters();

        let still_there = self
            .open_adapter
            .as_ref()
            .is_some_and(|open| self.adapters.contains(open));
        if !still_there {
            self.close();
        }
        if self.usb0.is_some() {
            return;
        }

        let previous = self.link;
        match choose_adapter(&self.adapters, self.selected.as_deref()) {
            AdapterChoice::One(adapter) => match self.config.open(adapter) {
                Ok(port) => {
                    self.usb0 = Some(port);
                    self.open_adapter = Some(adapter.clone());
                    self.open_error = None;
                    self.link = Link::Open;
                }
                Err(e) => {
                    self.open_error = Some(e.to_string());
                    self.link = Link::OpenFailed;
                }
            },
            AdapterChoice::None => self.link = Link::NoAdapter,
            AdapterChoice::Ambiguous => self.link = Link::Ambiguous,
            AdapterChoice::SelectedMissing => self.link = Link::SelectedMissing,
        }
        if self.link != previous {
            let _ = writeln!(std::io::stderr(), "Feetech bus: {}", self.link.as_str());
        }
    }

    /// Drive the adapter with this iSerial from now on.
    pub fn select(&mut self, serial: String) {
        if self.open_adapter.as_ref().and_then(|a| a.serial.as_deref()) != Some(serial.as_str()) {
            self.close();
        }
        self.selected = Some(serial);
        self.refresh();
    }

    fn close(&mut self) {
        self.usb0 = None;
        self.open_adapter = None;
    }
}
