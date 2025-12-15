use core::fmt::{self, Debug};

pub enum SpiError<TSPIERR> {
    Write(TSPIERR),
    Transfer(TSPIERR),
}

impl<TSPIERR: Debug> Debug for SpiError<TSPIERR> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Write(err) => write!(f, "Write({:?})", err),
            Self::Transfer(err) => write!(f, "Transfer({:?})", err),
        }
    }
}

pub enum PinError<TOPERR, TIPERR> {
    Output(TOPERR),
    Input(TIPERR),
}

impl<TOPERR: Debug, TIPERR: Debug> Debug for PinError<TOPERR, TIPERR> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Output(err) => write!(f, "Output({:?})", err),
            Self::Input(err) => write!(f, "Input({:?})", err),
        }
    }
}

pub enum SxError<TSPIERR, TOPERR, TIPERR> {
    Spi(SpiError<TSPIERR>),
    Pin(PinError<TOPERR, TIPERR>),
    Timeout,
    BufferTooSmall,
}

impl<TSPIERR: Debug, TOPERR: Debug, TIPERR: Debug> Debug for SxError<TSPIERR, TOPERR, TIPERR> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Spi(err) => write!(f, "Spi({:?})", err),
            Self::Pin(err) => write!(f, "Pin({:?})", err),
            Self::Timeout => write!(f, "Timeout"),
            Self::BufferTooSmall => write!(f, "BufferTooSmall"),
        }
    }
}

impl<TSPIERR, TOPERR, TIPERR> From<SpiError<TSPIERR>> for SxError<TSPIERR, TOPERR, TIPERR> {
    fn from(spi_err: SpiError<TSPIERR>) -> Self {
        SxError::Spi(spi_err)
    }
}

impl<TSPIERR, TOPERR, TIPERR> From<PinError<TOPERR, TIPERR>> for SxError<TSPIERR, TOPERR, TIPERR> {
    fn from(spi_err: PinError<TOPERR, TIPERR>) -> Self {
        SxError::Pin(spi_err)
    }
}
