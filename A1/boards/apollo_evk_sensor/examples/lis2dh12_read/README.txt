Configures the lis2dh12 to sample at 100Hz and set its watermark based
on the LIS2DH12_SAMPLE_SIZE define. When the lis2dh12 FIFO hits
its watermark (data-ready if sample size equals 1), an interrupt line
rises causing the Apollo MCU to interrupt and begin draining the FIFO while
sleeping and periodically waking to empty the internal IOM. The samples are
plotted over the ITM. Use AM FLash to view the real-time plot.

Note: No ITM debug printing takes place in this example.

