function Decoder(bytes, port) {
  var decoded = {};
  var events = {
    1: 'ok',
    2: 'error',
  };
  console.log(bytes)
  decoded.event = events[port];
  decoded.height = (bytes[0] << 8) + bytes[1];
  decoded.theft = (bytes[2] & 1);
  decoded.lid_open = ((bytes[2] & 2) >> 1);
  decoded.distance_sensor = ((bytes[2] & 4) >> 2);
  return decoded;
}
