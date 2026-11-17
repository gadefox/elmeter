int main(void) {
  meter_init();
  for (;;) {
    meter_poll();
    // UART: SETTIME -> meter_set_time(), dump -> meter_read_page()
  }
}
