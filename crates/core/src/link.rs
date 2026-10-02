use core::marker::PhantomData;
use core::result::Result;

use crate::serial::SerialChannel;

pub struct Disconnected;
pub struct Connected;

pub struct Link<State, C: SerialChannel> {
    _state: PhantomData<State>,
    pub channel: C,
}

impl<C: SerialChannel> Link<Disconnected, C> {
    pub fn new(channel: C) -> Self {
        Self {
            _state: PhantomData,
            channel,
        }
    }

    pub fn connect(mut self) -> Result<Link<Connected, C>, (Link<Disconnected, C>, C::Error)> {
        if let Err(e) = self.channel.open() {
            return Err((Link {
                _state: PhantomData,
                channel: self.channel,
            }, e));
        }

        Ok(Link {
            _state: PhantomData,
            channel: self.channel,
        })
    }
}

impl<C: SerialChannel> Link<Connected, C> {
    pub fn disconnect(mut self) -> Link<Disconnected, C> {
        let _ = self.channel.close();
        Link {
            _state: PhantomData,
            channel: self.channel,
        }
    }
}

pub enum AnyLink<C: SerialChannel> {
    Disconnected(Link<Disconnected, C>),
    Connected(Link<Connected, C>),
}

impl<C: SerialChannel> AnyLink<C> {
    pub fn as_connected_mut(&mut self) -> Option<&mut Link<Connected, C>> {
        match self {
            AnyLink::Connected(link) => Some(link),
            _ => None,
        }
    }
}

pub struct Controller<C: SerialChannel> {
    pub link: Option<AnyLink<C>>
}

impl<C: SerialChannel> Controller<C> {
    pub fn tick(&mut self) {
        let link = self.link.take().unwrap();
        self.link = Some(match link {
            AnyLink::Disconnected(link) => {
                match link.connect() {
                    Ok(connected) => AnyLink::Connected(connected),
                    Err((link, _)) => AnyLink::Disconnected(link)
                }
            }
            AnyLink::Connected(link) => AnyLink::Connected(link),
        });
    }
}
