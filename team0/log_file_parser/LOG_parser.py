# 4 byte float roll_deg
# 4 byte float pitch_deg
# 4 byte float ambient_pressure_hpa
# 4 byte float longitude_deg
# 4 byte float latitude_deg
# 1 byte bool servo_triggered
import struct

with open("log39.dat", "rb") as f:
    while True:
      roll_deg_byte = f.read(4)
      if roll_deg_byte == b"":
          print("END OF FILE")
          break
      pitch_deg_byte = f.read(4)
      ambient_pressure_hpa_byte = f.read(4)
      longitude_deg_byte = f.read(4)
      latitude_deg_byte = f.read(4)
      servo_triggered_byte = f.read(1)

      [roll_deg] = struct.unpack('f', roll_deg_byte)
      [pitch_deg] = struct.unpack('f', pitch_deg_byte)
      [ambient_pressure_hpa] = struct.unpack('f', ambient_pressure_hpa_byte)
      [longitude_deg] = struct.unpack('f', longitude_deg_byte)
      [latitude_deg] = struct.unpack('f', latitude_deg_byte)
      [servo_triggered] = struct.unpack('b', servo_triggered_byte)

      print(roll_deg, pitch_deg, ambient_pressure_hpa, longitude_deg, latitude_deg, servo_triggered)


