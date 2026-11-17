#![allow(unexpected_cfgs)]
#![feature(const_ops)]
#![feature(const_trait_impl)]

use embassy_time::Timer;
pub use esp_idf_matter::matter::dm::clusters::decl::door_lock;
use esp_idf_matter::matter::with;
use std::{
    cell::{Cell, RefCell},
    sync::{
        atomic::{AtomicBool, AtomicU8, Ordering},
        Arc,
    },
    time::{Duration, Instant},
};

use esp_idf_svc::hal::gpio::{InputOutput, Level, Output, PinDriver};
use rs_matter::{
    dm::{Dataver, EndptId, HandlerContext, InvokeContext, ReadContext, WriteContext},
    error::{Error, ErrorCode},
    tlv::{Nullable, TLVBuilderParent},
};

use crate::door_lock::*;

#[derive(Debug, Copy, Clone)]
enum LockState {
    Locked,
    UnlockedUntil(Instant),
}

/// Connectivity shown by the status LED. This is not a commissioning-complete
/// signal: a fabric can exist before the commissioner finishes setup.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum NetStatus {
    Booting = 0,
    /// No fabric is configured; setup may still be in progress.
    Commissioning = 1,
    /// A fabric exists, but the operational network is unavailable.
    WifiConnecting = 2,
    /// A fabric exists and the operational network has an IPv6 address.
    Online = 3,
}

impl NetStatus {
    pub fn from_link(has_fabric: bool, link_ready: bool) -> Self {
        match (has_fabric, link_ready) {
            (false, _) => Self::Commissioning,
            (true, false) => Self::WifiConnecting,
            (true, true) => Self::Online,
        }
    }

    pub fn store(self, slot: &AtomicU8) {
        slot.store(self as u8, Ordering::Relaxed);
    }

    fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::Commissioning,
            2 => Self::WifiConnecting,
            3 => Self::Online,
            _ => Self::Booting,
        }
    }
}

/// GPIO23: solid during boot, wink for setup, fast blink for WiFi, heartbeat online.
pub async fn run_status_led(mut led: PinDriver<'static, Output>, net: Arc<AtomicU8>) {
    const TICK_MS: u64 = 50;
    let mut t: u64 = 0;
    loop {
        let on = match NetStatus::from_u8(net.load(Ordering::Relaxed)) {
            NetStatus::Booting => true,
            NetStatus::Commissioning => t % 1500 >= 150,
            NetStatus::WifiConnecting => (t / 150) % 2 == 0,
            NetStatus::Online => t % 2000 < TICK_MS,
        };
        let _ = led.set_level(if on { Level::High } else { Level::Low });
        Timer::after_millis(TICK_MS).await;
        t = t.wrapping_add(TICK_MS);
    }
}

type Relay = PinDriver<'static, InputOutput>;

pub struct SolenoidHandler {
    dataver: Dataver,
    endpoint_id: EndptId,
    state: Cell<LockState>,
    relays: RefCell<(Relay, Relay)>,
    unlock_request: Arc<AtomicBool>,
}

impl SolenoidHandler {
    pub fn new(
        dataver: Dataver,
        endpoint_id: EndptId,
        relay_a: Relay,
        relay_b: Relay,
        unlock_request: Arc<AtomicBool>,
    ) -> Self {
        Self {
            dataver,
            endpoint_id,
            state: Cell::new(LockState::Locked),
            relays: RefCell::new((relay_a, relay_b)),
            unlock_request,
        }
    }

    fn notify(&self, ctx: &impl HandlerContext) {
        self.dataver_changed();
        ctx.notify_attr_changed(
            self.endpoint_id,
            Self::CLUSTER.id,
            AttributeId::LockState as _,
        );
    }
}

const UNLOCK_TIME: Duration = Duration::from_secs(5);

// Required trait methods for commands this lock does not expose.
macro_rules! unsupported_commands {
    ($($name:ident $(<$p:ident: $bound:path>)? ($($ty:ty),* $(,)?) -> $ret:ty;)*) => {$(
        async fn $name $(<$p: $bound>)? (
            &self, _: impl InvokeContext, $(_: $ty),*
        ) -> Result<$ret, Error> {
            Err(ErrorCode::InvalidAction.into())
        }
    )*};
}

impl ClusterAsyncHandler for SolenoidHandler {
    const CLUSTER: rs_matter::dm::Cluster<'static> = FULL_CLUSTER
        .with_revision(7)
        .with_attrs(with!(required; AttributeId::AutoRelockTime))
        .with_cmds(with!(
            CommandId::UnlockWithTimeout | CommandId::UnlockDoor | CommandId::LockDoor
        ));

    fn dataver(&self) -> u32 {
        self.dataver.get()
    }

    fn dataver_changed(&self) {
        self.dataver.changed();
    }

    async fn run(&self, ctx: impl HandlerContext) -> Result<(), Error> {
        self.notify(&ctx);

        loop {
            if self.unlock_request.swap(false, Ordering::Relaxed) {
                log::info!("External unlock request received");
                self.state
                    .set(LockState::UnlockedUntil(Instant::now() + UNLOCK_TIME));
            }

            let new_relay_state = match self.state.get() {
                LockState::Locked => Level::Low,
                LockState::UnlockedUntil(time) if time < Instant::now() => {
                    self.state.set(LockState::Locked);
                    Level::Low
                }
                LockState::UnlockedUntil(_) => Level::High,
            };

            if let Ok(ref mut itm) = self.relays.try_borrow_mut() {
                if itm.0.get_level() != new_relay_state {
                    self.notify(&ctx);
                }

                itm.0.set_level(new_relay_state).expect("set relay A");
                itm.1.set_level(new_relay_state).expect("set relay B");
            }

            Timer::after_millis(100).await;
        }
    }

    async fn auto_relock_time(&self, _: impl ReadContext) -> Result<u32, Error> {
        Ok(UNLOCK_TIME.as_secs() as u32)
    }

    async fn set_auto_relock_time(&self, _: impl WriteContext, _: u32) -> Result<(), Error> {
        Err(ErrorCode::InvalidAction.into())
    }

    async fn lock_state(&self, _: impl ReadContext) -> Result<Nullable<DlLockState>, Error> {
        Ok(Nullable::some(match self.state.get() {
            LockState::Locked => DlLockState::Locked,
            LockState::UnlockedUntil(_) => DlLockState::Unlocked,
        }))
    }

    async fn lock_type(&self, _: impl ReadContext) -> Result<DlLockType, Error> {
        Ok(DlLockType::Magnetic)
    }

    async fn actuator_enabled(&self, _: impl ReadContext) -> Result<bool, Error> {
        Ok(true)
    }

    async fn operating_mode(&self, _: impl ReadContext) -> Result<OperatingModeEnum, Error> {
        Ok(OperatingModeEnum::Normal)
    }

    async fn supported_operating_modes(
        &self,
        _: impl ReadContext,
    ) -> Result<DlSupportedOperatingModes, Error> {
        Ok(DlSupportedOperatingModes::NORMAL)
    }

    async fn set_operating_mode(
        &self,
        _: impl WriteContext,
        _: OperatingModeEnum,
    ) -> Result<(), Error> {
        Err(ErrorCode::InvalidAction.into())
    }

    async fn handle_lock_door(
        &self,
        _: impl InvokeContext,
        _: LockDoorRequest<'_>,
    ) -> Result<(), Error> {
        self.state.set(LockState::Locked);
        Ok(())
    }

    async fn handle_unlock_door(
        &self,
        _: impl InvokeContext,
        _: UnlockDoorRequest<'_>,
    ) -> Result<(), Error> {
        self.state
            .set(LockState::UnlockedUntil(Instant::now() + UNLOCK_TIME));
        Ok(())
    }

    async fn handle_unlock_with_timeout(
        &self,
        _: impl InvokeContext,
        request: UnlockWithTimeoutRequest<'_>,
    ) -> Result<(), Error> {
        let duration = request
            .timeout()
            .map(|sec| Duration::from_secs(sec as u64))
            .unwrap_or(UNLOCK_TIME);
        self.state
            .set(LockState::UnlockedUntil(Instant::now() + duration));
        Ok(())
    }

    unsupported_commands! {
        handle_set_week_day_schedule(SetWeekDayScheduleRequest<'_>) -> ();
        handle_get_week_day_schedule<P: TLVBuilderParent>(GetWeekDayScheduleRequest<'_>, GetWeekDayScheduleResponseBuilder<P>) -> P;
        handle_clear_week_day_schedule(ClearWeekDayScheduleRequest<'_>) -> ();
        handle_set_year_day_schedule(SetYearDayScheduleRequest<'_>) -> ();
        handle_get_year_day_schedule<P: TLVBuilderParent>(GetYearDayScheduleRequest<'_>, GetYearDayScheduleResponseBuilder<P>) -> P;
        handle_clear_year_day_schedule(ClearYearDayScheduleRequest<'_>) -> ();
        handle_set_holiday_schedule(SetHolidayScheduleRequest<'_>) -> ();
        handle_get_holiday_schedule<P: TLVBuilderParent>(GetHolidayScheduleRequest<'_>, GetHolidayScheduleResponseBuilder<P>) -> P;
        handle_clear_holiday_schedule(ClearHolidayScheduleRequest<'_>) -> ();
        handle_set_user(SetUserRequest<'_>) -> ();
        handle_get_user<P: TLVBuilderParent>(GetUserRequest<'_>, GetUserResponseBuilder<P>) -> P;
        handle_clear_user(ClearUserRequest<'_>) -> ();
        handle_set_credential<P: TLVBuilderParent>(SetCredentialRequest<'_>, SetCredentialResponseBuilder<P>) -> P;
        handle_get_credential_status<P: TLVBuilderParent>(GetCredentialStatusRequest<'_>, GetCredentialStatusResponseBuilder<P>) -> P;
        handle_clear_credential(ClearCredentialRequest<'_>) -> ();
        handle_unbolt_door(UnboltDoorRequest<'_>) -> ();
        handle_set_aliro_reader_config(SetAliroReaderConfigRequest<'_>) -> ();
        handle_clear_aliro_reader_config() -> ();
    }
}
