This example application implements the standard BLE HRP profile using the
ExactLE stack and the Dialog DA14581 BLE radio. This application is able to
communicate with standard heart-rate applications running on recent model
mobile devices.

In this example implementation, the heart rate value is reported as a
constant "78", and the "kCals consumed" value is reported as a single
incrementing integer value. In a real application, these values could be
supplied by a heart-rate sensor and context-tracking software.

