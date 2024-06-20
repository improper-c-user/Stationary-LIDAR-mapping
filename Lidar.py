import serial
import numpy as np
import warnings
""" Functions:
1. Confirm for header 
2. Confirm command type 
3. Comfirm packet size  
4. Get First and End angles and Sample data 
5. Calculate and store distance, angle respectively

Lidar Packet structure: (LSB = least significant bit, MSB = Most significant bit)
1. header
2. CMD = Commnand Packet
3. PCKSZ = Packet size
4. FSA = First Sample Angle
5. LSA = Last Samplee Anlge
6. CS = Checksum
7. S1-Sn = Distance Sample

Bit 0      2         4    
   |HEADER |CMD|PCKSZ|  FSA  |  LSA  |  CS   |  S1   |.....|  Sn   |
   |LSB|MSB|LSB| MSB |LSB|MSB|LSB|MSB|LSB|MSB|LSB|MSB|.....|LSB|MSB|

"""

def Twobyte_XOR(checksum, bytes):
   CMD_PCKSZ = 0x0000
   LSA = 0x0000
   for i in range(0,len(bytes) - 1,2):
      if i == 2:
         CMD_PCKSZ = (bytes[i+1]<<8) | bytes[i]
         continue
      elif i == 6:
         LSA = (bytes[i+1]<<8 )| bytes[i]
         continue
      elif i == 8:
         continue
      else:
         combined_bytes = (bytes[i+1] << 8) | bytes[i]
         checksum ^= combined_bytes
   checksum = (checksum ^ CMD_PCKSZ) ^ LSA
   return checksum

def ToDistance(hightBytes, lowBytes):
   hightBytes &= 0xFF
   lowBytes &= 0xFF
   combined = (hightBytes << 8) | lowBytes
   return float(combined/4)

def ToAngle(distance, angle_FSA, angle_LSA, angle_num):
   angle_fstage = 0
   if angle_num == 0:
      angle_fstage = angle_FSA
   elif angle_num == 39:
      angle_fstage = angle_LSA
   else:
      if angle_LSA >= angle_FSA:
         angle_fstage = ((angle_LSA - angle_FSA)/(40-1)) * (angle_num-1) + angle_FSA
      else:
         angle_fstage = ((angle_LSA + 360 - angle_FSA)/(40-1)) * (angle_num-1) + angle_FSA
   if distance == 0:
      angleCorrect = 0
   else:
      angleCorrect = np.arctan(21.8* (155.3 - distance) / (155.3 * distance))
   return (angle_fstage + angleCorrect)
   
class Lidar:
   def __init__(self, packetNumber = 10):
      self.Serial = serial.Serial("COM3",115200, timeout=1)
      self.define_header = 0x55AA
      self.define_cmd_type = 0x00
      self.define_packet_Size = 0x28
      self.number_packet = 10
      if packetNumber == 10:
         self.cmd_packet = [0x2A, 0x42, 0x22, 0x9A, 0x86, 0x20, 0xCC, 0x00, 0x9C, 0x00]

   def get_lidar_scan(self, next_cmd):
      scans_dist = []
      scans_ang = []
      buffer = []
      angle_bytes = []
      checksum_bytes = []
      distances_bytes = []
      cmd_packet = self.cmd_packet[next_cmd]

      header = 0x0000
      cmd_type = 0x01
      checksum = 0x0000
      flagged = False

      byte = self.Serial.read(180)
      buffer.append(byte)
      while True:
         if len(buffer) > 180:
            buffer.pop(0)
            header = (buffer[1]<<8) | buffer[0]
            cmd_type = buffer[2] & 0x01
            packet_size = buffer[3]
            flagged = (header == self.define_header and buffer[2] == cmd_packet and cmd_type == self.define_cmd_type and packet_size == self.define_packet_Size)
         else:
            byte = self.Serial.read(1)
            buffer.append(byte[0])
         
         while flagged:
            # print(f"Command: {cmd_type}, Size: {buffer[3]}, Packet:{hex(header)} {hex(buffer[2])} {hex(buffer[3])}")
            # Parsing Data BEGIN;
            angle_bytes = buffer[4:8]
            checksum_bytes = buffer[8:10]
            distances_bytes = buffer[10:90]
            # Parsing Data END;
            
            #BEGIN Checksum;
            rcv_checksum = (checksum_bytes[1]<<8) | checksum_bytes[0]
            checksum = Twobyte_XOR(checksum, buffer[:90])
            # print(f"Rcv: {hex(checksum_bytes[0])} {hex(checksum_bytes[1])} -> {hex(rcv_checksum)}, Check: {hex(checksum)}")
            if True:
               buffer = buffer[90:]
               #print(f"Comfirmed Checksum: {hex(checksum)}")
               angle_FSA = (((angle_bytes[1] <<8) | angle_bytes[0])>>1)/64
               angle_LSA = (((angle_bytes[3] <<8) | angle_bytes[2])>>1)/64
               # print(f" FSA: {angle_FSA}, LSA: {angle_LSA}")
               for i in range(0, len(distances_bytes)-1, 2):
                  distance = ToDistance(distances_bytes[i+1], distances_bytes[i])
                  angle = ToAngle(distance, angle_FSA, angle_LSA, i/2)
                  if angle > 360:
                     angle -= 360
                  scans_dist.append(distance) #distance, angle
                  scans_ang.append(angle)
                  # print(f"Distance: {distance}, Angle: {angle}")
            #END Checksum;
                  #distance, angle
               checksum = 0x0000
               angle_bytes.clear()
               checksum_bytes.clear()
               distances_bytes.clear() 
               if len(scans_ang) == 80:
                  return scans_dist, scans_ang
            else:
               warnings.warn("Checksum Error!", RuntimeWarning)
               return self.get_lidar_scan(next_cmd)