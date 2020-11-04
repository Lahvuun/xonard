project('xonard', 'c')

# 1.0.22 introduced libusb_set_option()
usbdep = dependency('libusb-1.0', version : '>=1.0.22')

flags = ['-Wall','-Wextra', '-Wpedantic', '-Wconversion']
executable('xonard',
  'src/main.c',
  c_args : flags,
  dependencies : usbdep,
  install: true)
