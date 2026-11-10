use std::sync::atomic::{AtomicBool, Ordering};

static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn request_stop(_: libc::c_int) {
    STOP.store(true, Ordering::SeqCst);
}

pub fn install_stop_handler() {
    for sig in [libc::SIGINT, libc::SIGTERM, libc::SIGHUP, libc::SIGQUIT] {
        let prev = unsafe { libc::signal(sig, request_stop as *const () as libc::sighandler_t) };
        if prev == libc::SIG_IGN {
            unsafe { libc::signal(sig, libc::SIG_IGN) };
        }
    }
}

pub fn stop_requested() -> bool {
    STOP.load(Ordering::SeqCst)
}
