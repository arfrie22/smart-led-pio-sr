#![no_std]
#![feature(generic_const_exprs)]
//! [ws2812](https://www.sparkfun.com/datasheets/LCD/HD44780.pdf)
//! https://learn.adafruit.com/neopio-drive-lots-of-leds-with-raspberry-pi-pico/code-walkthrough-pio

use embassy_time::Timer;
use fixed::types::U24F8;
use smart_leds::RGB8;

use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::dma::{AnyChannel, Channel};
use embassy_rp::pio::{
    Common, Config, FifoJoin, Instance, LoadedProgram, PioPin, ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_rp::{into_ref, Peripheral, PeripheralRef};

/// This struct represents a ws2812 program loaded into pio instruction memory.
pub struct PioWs2812SRProgram<'a, PIO: Instance> {
    prg: LoadedProgram<'a, PIO>,
}

// .program neopio
// .side_set 2 opt

// .wrap_target
//     set x, 7            side 2
//     pull

// bitloop0:
//     set pins, 1         side 0
//     jmp x--, bitloop0   side 1
//     set x, 7            side 2
// bitloop1:
//     out pins, 1         side 0
//     jmp x--, bitloop1   side 1
//     set x, 7            side 2
// bitloop2:
//     set pins, 0         side 0
//     jmp x--, bitloop2   side 1
// .wrap

impl<'a, PIO: Instance> PioWs2812SRProgram<'a, PIO> {
    /// Load the ws2812 program into the given pio
    pub fn new(common: &mut Common<'a, PIO>) -> Self {
        let prg = pio_proc::pio_asm!(
            r#"
                .side_set 2 opt

                .wrap_target
                    set x, 7            side 2
                    pull

                bitloop0:
                    set pins, 1         side 0
                    jmp x--, bitloop0   side 1
                    set x, 7            side 2
                bitloop1:
                    out pins, 1         side 0
                    jmp x--, bitloop1   side 1
                    set x, 7            side 2
                bitloop2:
                    set pins, 0         side 0
                    jmp x--, bitloop2   side 1
                .wrap
            "#
        );

        let prg = common.load_program(&prg.program);

        Self { prg }
    }
}

/// Pio backed ws2812 driver
/// Const N is the number of ws2812 leds attached to this pin
pub struct PioWs2812SR<'d, P: Instance, const S: usize, const N: usize, const C: usize> {
    dma: PeripheralRef<'d, AnyChannel>,
    sm: StateMachine<'d, P, S>,
}

fn matrix_transpose(a: [u8; 8]) -> [u8; 8] {
    let mut b = [0; 8];

    // Load the array and pack it into x and y. 

    let mut x: u32 = ((a[0] as u32) <<24) | ((a[1] as u32) <<16) | ((a[2] as u32) <<8) | a[3] as u32; 
    let mut y: u32 = ((a[4] as u32) <<24) | ((a[5] as u32) <<16) | ((a[6] as u32) <<8) | a[7] as u32; 

    let mut t = (x ^ (x >> 7)) & 0x00AA00AA;  x = x ^ t ^ (t << 7); 
    t = (y ^ (y >> 7)) & 0x00AA00AA;  y = y ^ t ^ (t << 7); 

    t = (x ^ (x >>14)) & 0x0000CCCC;  x = x ^ t ^ (t <<14); 
    t = (y ^ (y >>14)) & 0x0000CCCC;  y = y ^ t ^ (t <<14); 

    t = (x & 0xF0F0F0F0) | ((y >> 4) & 0x0F0F0F0F); 
    y = ((x << 4) & 0xF0F0F0F0) | (y & 0x0F0F0F0F); 
    x = t; 

    b[0] = (x>>24) as u8;
    b[1] = (x>>16) as u8;
    b[2] = (x>>8) as u8;
    b[3] = x as u8;

    b[4] = (y>>24) as u8;
    b[5] = (y>>16) as u8;
    b[6] = (y>>8) as u8;
    b[7] = y as u8;

    b
}

impl<'d, P: Instance, const S: usize, const N: usize, const C: usize> PioWs2812SR<'d, P, S, N, C> 
where [(); 8*N*3]: Sized {
    /// Configure a pio state machine to use the loaded ws2812 program.
    pub fn new(
        pio: &mut Common<'d, P>,
        mut sm: StateMachine<'d, P, S>,
        dma: impl Peripheral<P = impl Channel> + 'd,
        data: impl PioPin,
        clock: impl PioPin,
        strobe: impl PioPin,
        program: &PioWs2812SRProgram<'d, P>,
    ) -> Self {
        into_ref!(dma);

        // Setup sm0
        let mut cfg = Config::default();

        // Pin config
        let out_data = pio.make_pio_pin(data);
        let out_clock = pio.make_pio_pin(clock);
        let out_strobe = pio.make_pio_pin(strobe);

        sm.set_pin_dirs(embassy_rp::pio::Direction::Out, &[&out_data, &out_clock, &out_strobe]);
        cfg.set_out_pins(&[&out_data]);
        cfg.set_set_pins(&[&out_data, &out_clock, &out_strobe]);

        cfg.use_program(&program.prg, &[&out_clock, &out_strobe]);

        // Clock config, measured in kHz to avoid overflows
        let clock_freq = U24F8::from_num(clk_sys_freq() / 1000);
        let freq = U24F8::from_num(800 * 52);
        cfg.clock_divider = clock_freq / freq;


        // FIFO config
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 8,
            direction: ShiftDirection::Left,
        };

        sm.set_config(&cfg);
        sm.set_enable(true);

        Self {
            dma: dma.map_into(),
            sm,
        }
    }

    /// Write a buffer of [smart_leds::RGB8] to the ws2812 string
    pub async fn write(&mut self, colors: &[[RGB8; N]; C]) {
        // Precompute the word bytes from the colors
        // let mut words = [[0u32; N]; 3];
        let mut words = [0u32; 8*N*3];
        let mut word_index = 0;
        for i in 0..N {
            let mut r = [
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0
            ];
            for c in 0..C {
                r[7-c] = colors[c][i].r;
            }

            let r = matrix_transpose(r);

            let mut g = [
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0
            ];
            for c in 0..C {
                g[7-c] = colors[c][i].g;
            }
            let g = matrix_transpose(g);

            let mut b = [
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0
            ];
            for c in 0..C {
                b[7-c] = colors[c][i].b;
            }
            let b = matrix_transpose(b);

            let colors = [g, r, b];

            for c in colors {
                for i in c {
                    let word = (i as u32) << 24;
                    words[word_index] = word;
                    word_index += 1;
                }
            }

        }

        // DMA transfer
        self.sm.tx().dma_push(self.dma.reborrow(), &words).await;

        Timer::after_micros(55).await;
    }
}