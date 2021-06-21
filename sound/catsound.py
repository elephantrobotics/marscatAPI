import numpy as np
import os
import sys
sys.path.append(".")

class CatSound():
    def __init__(self):
        pass

    def meow(self, async_play=True):
        self.play_sound('meow-normal-2s.wav', async_play)

    def purr(self, async_play=True):
        self.play_sound('purr-6s.mp3', async_play)

    def angry(self, async_play=True):
        self.play_sound('meow-angry-4s.mp3', async_play)

    def eat(self, async_play=True):
        self.play_sound('eat-9s.mp3', async_play)

    def unhappy(self, async_play=True):
        self.play_sound('meow-unhappy-1s.wav', async_play)

    def meow_draw_attention(self, async_play=True):
        self.play_sound('meow-normal-draw-attention-1s.wav', async_play)

    def play_sound(self, sound_name, async_play=True):
        async_suffix = ''
        if async_play == True:
            async_suffix = ' &'

        volumn_type_list = ['quiet', 'medium', 'loud']
        meow_exclude_list = ['eat', 'purr']

        #volumn quiet medium loud
        volumn_adjust_type = volumn_type_list[int(np.random.random() * 3)]
        if (volumn_adjust_type == 'medium'
            ) or sound_name.split('-')[0] in meow_exclude_list:
            sound_adjust_name = sound_name
        else:
            sound_adjust_name = sound_name.split('.')[
                0] + '-' + volumn_adjust_type + '.' + sound_name.split(
                    '.')[1]

        #speed 90%-110%
        #speed_adjust_percent = round(np.random.randint(90, high=110) * 0.01, 2)
        #temp_sound_adjust_name = sound_adjust_name.split('.')[0] + '-temp' + '.' + sound_adjust_name.split('.')[1]

        #audio.a_speed(SOUNDS_PATH + sound_adjust_name, speed_adjust_percent, SOUNDS_PATH + temp_sound_adjust_name)
        print(sound_adjust_name)
        os.system('omxplayer -o local sound/sounds/' + sound_adjust_name +
                    ' >/dev/null' + async_suffix)

