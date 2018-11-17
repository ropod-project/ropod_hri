from os.path import join
import wave
import pyaudio
import yaml
import rospy
from ropod_ros_msgs.msg import StateInfo

class SoundCommunicator(object):
    def __init__(self):
        self.sound_config = rospy.get_param('~sound_config', '')
        self.sound_collection = rospy.get_param('~sound_collection', 'willow-sound')
        self.sound_request_topic = rospy.get_param('~sound_request_topic', '/state_info')

        self.sound_file_dictionary = dict()
        with open(self.sound_config, 'r') as sound_config_file:
            self.sound_file_dictionary = yaml.load(sound_config_file)

        self.audio_manager = pyaudio.PyAudio()
        rospy.Subscriber(self.sound_request_topic, StateInfo, self.sound_cb)

    def sound_cb(self, msg):
        sound = msg.state
        try:
            sound_file = self.sound_file_dictionary[sound]
        except:
            rospy.logerr('Sound %s does not exists', sound)
            return

        sound_file_path = join(self.sound_collection, sound_file)
        self.__play_sound(sound_file_path)

    def __play_sound(self, sound_file):
        try:
            wavefile = wave.open(sound_file, 'rb')
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
