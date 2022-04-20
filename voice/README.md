<!--
Copyright (c) 2019 Elephant Robotics, Inc. All rights reserved.

Using this MarsAI source code is subject to the terms and conditions of Apache 2.0 License. Check LICENSE for more information
-->

# Voice Recognition

## Library - pocketSphinx
You can download CMU pocket sphinx from [here](https://github.com/cmusphinx/pocketsphinx)

Support recognition of three languages: English, Chinese

- >Supported directives are stored in `corpus/corpus.txt`

### speak_config

- **Prototype**: `speak_config(language)`

- **Description**: Initialize the speech recognition language library.

- **Parameters**

  - `language` : language to be recognized (`English`or `Chinese`).
- **Return**
  - voice decoder.

### load_word

- **Prototype**: `load_word()`

- **Description**: Get Supported Instructions.

- **Return**
  - All supported commands.

### get_speak_queue

**Prototype**: `get_speak_queue()`

- **Description**: construct a queue.

- **Return**
  - queue.

### get_commands

**Prototype**: `get_commands(speak_queues)`

- **Description**: Get the value in the queue.

- **Parameters**

  - `speak_queues` : queue.

- **Return**
  - `value`: recognized words.
  - `None`: The specified command is not recognized



### speak_monitor

- **Prototype**: `speak_monitor(speak_queue, decoder, command)`

- **Description**: speech recognition function.

- **Parameters**

  - `speak_queue` : `queue`used to store recognized words.
  - `decoder`: voice decoder.
  - `command` : All supported commands.

## Instructions

```python
import threading
import voice.voice_base

vc = voice.voice_base.Voice()
command = vc.load_word()
decoder = vc.speak_config("English")

speak_queue = vc.get_speak_queue()

p = threading.Thread(target=vc.speak_monitor,
                        args=(speak_queue, decoder, command), daemon=True)
p.start()

while True:
    commands = vc.get_commands(speak_queue)
    if commands:
        print(commands)
```