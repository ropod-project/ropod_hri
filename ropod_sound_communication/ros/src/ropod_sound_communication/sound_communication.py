from os.path import join
import wave
import pyaudio
import yaml
import rospy
from ropod_ros_msgs.msg import StateInfo, Status
from ropod_ros_msgs.msg import TaskProgressGOTO, TaskProgressDOCK

class SoundCommunicator(object):
    def __init__(self):
        self.sound_config = rospy.get_param('~sound_config', '')
        self.sound_collection = rospy.get_param('~sound_collection', 'willow-sound')
        self.sound_request_topic = rospy.get_param('~sound_request_topic', '/state_info')
        self.go_to_progress_topic = rospy.get_param('~go_to_progress_topic', '/task_progress/goto')
        self.dock_progress_topic = rospy.get_param('~dock_progress_topic', '/task_progress/dock')

        self.sound_file_dictionary = dict()
        with open(self.sound_config, 'r') as sound_config_file:
            self.sound_file_dictionary = yaml.load(sound_config_file)

        self.audio_manager = pyaudio.PyAudio()
        self.state_info_sub = rospy.Subscriber(self.sound_request_topic,
                                               StateInfo,
                                               self.sound_cb)
        self.go_to_progress_sub = rospy.Subscriber(self.go_to_progress_topic,
                                                   TaskProgressGOTO,
                                                   self.go_to_progress_cb)
        self.dock_progress_sub = rospy.Subscriber(self.dock_progress_topic,
                                                  TaskProgressDOCK,
                                                  self.dock_progress_cb)

    def sound_cb(self, msg):
        sound = msg.state
        try:
            sound_file = self.sound_file_dictionary[sound]
            self.__play_sound(sound_file)
        except Exception as exc:
            rospy.logerr('[sound_communication/sound_cb] Sound %s does not exists', sound)
            rospy.logerr(str(exc))
            return

    def go_to_progress_cb(self, msg):
        try:
            if msg.status.status_code == Status.SUCCESS:
                self.__play_sound(self.sound_file_dictionary['success'])
            elif msg.status.status_code == Status.FAILED:
                self.__play_sound(self.sound_file_dictionary['error'])
            else:
                rospy.loginfo('[sound_communication/go_to_progress_cb] No sound for status %d',
                              msg.status.status_code)
        except Exception as exc:
            rospy.logerr('[sound_communication/go_to_progress_cb] %s', str(exc))
            return

    def dock_progress_cb(self, msg):
        try:
            if msg.status.status_code == Status.SUCCESS:
                self.__play_sound(self.sound_file_dictionary['success'])
            elif msg.status.status_code == Status.FAILED:
                self.__play_sound(self.sound_file_dictionary['error'])
            else:
                rospy.loginfo('[sound_communication/dock_progress_cb] No sound for status %d',
                              msg.status.status_code)
        except Exception as exc:
            rospy.logerr('[sound_communication/dock_progress_cb] %s', str(exc))
            return

    def __play_sound(self, sound_file):
        try:
            sound_file_path = join(self.sound_collection, sound_file)
            wavefile = wave.open(sound_file_path, 'rb')
            data = wavefile.readframes(1024)
            data_format = self.audio_manager.get_format_from_width(wavefile.getsampwidth())
            stream = self.audio_manager.open(format=data_format,
                                             channels=wavefile.getnchannels(),
                                             rate=wavefile.getframerate(),
                                             output=True)

            while len(data) > 0:
                stream.write(data)
                data = wavefile.readframes(1024)
            stream.stop_stream()
            stream.close()
        except Exception as exc:
            rospy.logerr('Error while playing sound %s: %s', sound_file, str(exc))
