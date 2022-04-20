#!/usr/bin/python3
# coding=utf-8

# Author:       Leonid, Joey


import audioop
import copy
import pocketsphinx.pocketsphinx
import pyaudio
import queue
import socket
import sphinxbase.sphinxbase
import threading
import json
import time
import logging



class Voice():
    def __init__(self):
        self.path = '/home/pi/marscatAPI/voice/corpus'
        self.stream = None
        pass

    def speak_config(self,language="English"):
        config = pocketsphinx.pocketsphinx.Decoder.default_config()
        if language == "English":
            config.set_string('-hmm', self.path + '/en-us')
            config.set_string('-lm', self.path + '/6247.lm')
            config.set_string('-dict', self.path + '/6247.dic')
        elif language == "Chinese":
            config.set_string('-hmm', self.path + '/zh-ch')
            config.set_string('-lm', self.path + '/5758.lm')
            config.set_string('-dict', self.path + '/5758.dic')
        elif language == "Japanese":
            config.set_string('-hmm', self.path + '/jp')
            config.set_string('-lm', self.path + '/9070.lm')
            config.set_string('-dict', self.path + '/9070.dic')


        config.set_string('-logfn', '/dev/null')
        decoder = pocketsphinx.pocketsphinx.Decoder(config)
        return decoder

    def load_word(self):
        try:
            file = open(self.path + "/corpus.txt", 'r')
        except FileNotFoundError:
            file = open('corpus.txt', 'r')
        commands = file.readlines()
        # removes spaces e.g. '\n'
        commands = [command.strip() for command in commands] 
        # print(commands)
        return commands

    def get_speak_queue(self):
        return queue.Queue(maxsize=1)

    def get_commands(self, speak_queues):
        if not speak_queues.empty():
            return speak_queues.get()
        return None

    def speak_monitor_test(self, speak_queue, decoder, command):
        p = pyaudio.PyAudio()

        stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8192)

        stream.start_stream()
        in_speech_bf = False
        decoder.start_utt()

        while True:
            buf = stream.read(1024, False)
            if buf:
                decoder.process_raw(buf, False, False)

                if decoder.get_in_speech() != in_speech_bf:
                    in_speech_bf = decoder.get_in_speech()
                    if not in_speech_bf:
                        decoder.end_utt()
                        try:
                            word = decoder.hyp().hypstr
                        except Exception:
                            word = ''
                        if word in command:
                            commands = {'type': 1, 'word': word}
                        else:
                            commands = {'type': 2, 'word': ''}
                        if not speak_queue.empty():
                            speak_queue.get()
                            speak_queue.task_done()
                        speak_queue.put(commands)
                        decoder.start_utt()
        decoder.end_utt()


    def speak_monitor(self, speak_queue, decoder, command):
        p = pyaudio.PyAudio()

        while True:
            try:
                stream = p.open(format=pyaudio.paInt16,
                                channels=1,
                                rate=16000,
                                input=True,
                                frames_per_buffer=8192)
                break
            except:
                time.sleep(1)
                continue

        stream.start_stream()
        in_speech_bf = False
        decoder.start_utt()

        while True:
            buf = stream.read(1024, False)
            if buf:
                decoder.process_raw(buf, False, False)

                buf_volumn = copy.deepcopy(buf)
                rms_data = audioop.rms(buf_volumn, 2)
                db = int(rms_data / 20) + 10

                if decoder.get_in_speech() != in_speech_bf:
                    in_speech_bf = decoder.get_in_speech()

                    if not in_speech_bf:
                        decoder.end_utt()
                        try:
                            word = decoder.hyp().hypstr
                        except Exception:
                            word = ''

                        print(f'first: ({word})')

                        ACTIVATION_WORDS = [
                            'HI MARSCAT', 'MARSCAT', 'MASSCAT', 'MASKCAT', 'MARS',
                            'ASSCAT', 'MASS', '咪咪', '小猫', '猫'
                        ]
                        ff = False
                        for item in ACTIVATION_WORDS:
                            if item in word:
                                ff = True

                        # if word in ACTIVA  TION_WORDS:
                        if ff:
                            print("voice wake-up")
                            commands = {
                                'type': 1,
                                'word': 'start_listen',
                                'db': db
                            }
                            speak_queue.put(commands)
                            decoder.start_utt()
                            start_time = time.time()
                            while True:
                                buf = stream.read(1024, False)
                                if buf:
                                    decoder.process_raw(buf, False, False)
                                    buf_volumn = copy.deepcopy(buf)
                                    rms_data = audioop.rms(buf_volumn, 2)
                                    db = int(rms_data / 20) + 10

                                    if decoder.get_in_speech() != in_speech_bf:
                                        in_speech_bf = decoder.get_in_speech()

                                        if not in_speech_bf:
                                            decoder.end_utt()
                                            try:
                                                word = decoder.hyp().hypstr
                                            except Exception:
                                                word = ''

                                            print(f'second: ({word})')

                                            if word in command and word not in ACTIVATION_WORDS:
                                                start_time = time.time()
                                                commands = {
                                                    'type': 1,
                                                    'word': word,
                                                    'db': db
                                                }
                                            else:
                                                commands = {
                                                    'type': 2,
                                                    'word': '',
                                                    'db': db
                                                }
                                            if not speak_queue.empty():
                                                speak_queue.get()
                                                speak_queue.task_done()
                                            speak_queue.put(commands)
                                            decoder.start_utt()
                                end_time = time.time()
                                if end_time - start_time > 17:
                                    commands = {
                                        'type': 1,
                                        'word': 'end_listen',
                                        'db': db
                                    }
                                    speak_queue.put(commands)
                                    decoder.end_utt()
                                    break

                        decoder.start_utt()


# Test voice data
def test_voice():
    vc = Voice()
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


if __name__ == "__main__":
    test_voice()
