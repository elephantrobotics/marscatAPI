<!--
Copyright (c) 2019 Elephant Robotics, Inc. All rights reserved.

Using this MarsAI source code is subject to the terms and conditions of Apache 2.0 License. Check LICENSE for more information
-->

# Sound

The sound is to control the loudspeaker inside MarsCat to play different audios.

## Functions

### meow

- **Prototype**: `meow()`

- **Description**: Play meow sounds.


### purr

- **Prototype**: `purr()`

- **Description**: Play purr sounds.


### purr

- **Prototype**: `purr()`

- **Description**: Play purr sounds.

### angry

- **Prototype**: `angry()`

- **Description**: Play angry sounds.

### eat

- **Prototype**: `eat()`

- **Description**: Play eat sounds.

### unhappy

- **Prototype**: `unhappy()`

- **Description**: Play unhappy sounds.

### meow_draw_attention

- **Prototype**: `meow_draw_attention()`

- **Description**: Play meow sounds.

### play_sound

- **Prototype**: `play_sound(sound_name)`

- **Description**: Play specified sound.

- **Parameters**

  - `sound_name` : Name of the audio.

## Instructions

### Play sounds

```python
import sound.catsound
import time

s = sound.catsound.CatSound()

s.meow()
time.sleep(2)

s.purr()
time.sleep(6)

s.angry()
time.sleep(4)

s.eat()
time.sleep(9)

s.unhappy()
time.sleep(1)

s.meow_draw_attention()
time.sleep(1)

"""
If you want to add a new sound file, add it to the "Sounds" folder.
One sound file corresponds to three files with different names.
like this:
sounds/new_file-quite.mp3
sounds/new_file-medium.mp3
sounds/new_file-loud.mp3

add new function:
def new_sound(self, async_play=True):
self.play_sound('new_file.mp3')
"""

s.play_sound('meow-normal-2s-loud.wav')

```
