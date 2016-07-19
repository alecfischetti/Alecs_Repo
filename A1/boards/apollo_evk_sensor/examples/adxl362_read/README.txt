Configures the adxl362 to sample at 100Hz and set its watermark based
on the ADXL362_SAMPLE_SIZE define. When the adxl362 FIFO hits its
watermark, an interrupt line asserts causing the Apollo MCU to interrupt
and begin draining the FIFO while sleeping and periodically waking to
empty the internal IOM. The samples are plotted over the ITM.

While moving the EVK board, use AM Flash (click "Show Plot Window") to
view the real-time plot.

Note: No ITM debug printing takes place in this example.

