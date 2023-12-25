import pyttsx3

text_drive = "Drive mode"
text_arm = "Arm mode"
text_wrist = "Wrist mode"
text_idle = "Idle mode"
text_cursor = "Cursor contro mode"
text_robot_calibration = "Look straight and click to calibrate"
text_cursor_calibration = "Look at screen center and click to calibrate"

engine = pyttsx3.init()
current_rate = engine.getProperty('rate')
engine.setProperty('rate', current_rate * 0.7)

engine.save_to_file(text_drive, 'drive.mp3')
engine.save_to_file(text_arm, 'arm.mp3')
engine.save_to_file(text_wrist, 'wrist.mp3')
engine.save_to_file(text_idle, 'idle.mp3')
engine.save_to_file(text_cursor, 'cursor.mp3')
engine.save_to_file(text_robot_calibration, 'robot_calibration.mp3')
engine.save_to_file(text_cursor_calibration, 'cursor_calibration.mp3')

engine.runAndWait()