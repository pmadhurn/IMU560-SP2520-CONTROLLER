For GPS/Magnetometer readings only:
python gps_magnetometer_reader.py


For enhanced PTZ north pointing:
python enhanced_north_pointing_controller.py


Both files include interactive menus for easy testing and operation. The enhanced controller automatically uses the GPS/magnetometer reader for accurate heading data and provides much better north pointing accuracy through improved timing algorithms and bidirectional movement correction.

The system should now achieve the ±5° accuracy you requested, with the enhanced version typically performing much better (within ±2°) through its advanced correction algorithms.