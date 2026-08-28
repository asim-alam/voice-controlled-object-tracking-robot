import socket
import threading
import queue
import time
import numpy as np
from collections import deque
import speech_recognition as sr
import tempfile
import os
import wave 
import requests
import json
import struct
from gtts import gTTS
import pygame
import io


ESP32_IP = os.getenv("VROD_ESP32_IP", "192.168.1.100")
PC_IP = os.getenv("VROD_PC_IP", "0.0.0.0")
AUDIO_RECEIVE_PORT = 12345  # Port to receive audio from ESP32
COMMAND_SEND_PORT = 12346   # Port to send commands back to ESP32
AUDIO_SEND_PORT = 12348     # Port to send audio response to ESP32

# Audio settings (must match ESP32 settings)
SAMPLE_RATE = 16000
I2S_READ_LEN = 2048  # Maximum UDP payload sent by the ESP32-S3 firmware
AUDIO_BUFFER_SECONDS = 4.0  # Buffer for better recognition
PROCESS_CHUNK_SECONDS = 2.5  # Process 2.5 second chunks

# Gemini AI Configuration
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "")
GEMINI_MODEL = os.getenv("GEMINI_MODEL", "gemini-2.5-flash")
GEMINI_URL = os.getenv(
    "GEMINI_URL",
    f"https://generativelanguage.googleapis.com/v1beta/models/{GEMINI_MODEL}:generateContent",
)

# Keyword mappings
MOTOR_COMMANDS = {
    'forward': 'FORWARD',
    'move forward': 'FORWARD',
    'go forward': 'FORWARD',
    'ahead': 'FORWARD',
    
    'backward': 'BACKWARD',
    'move backward': 'BACKWARD', 
    'go backward': 'BACKWARD',
    'back': 'BACKWARD',
    'reverse': 'BACKWARD',
    
    'left': 'LEFT',
    'turn left': 'LEFT',
    'go left': 'LEFT',
    
    'right': 'RIGHT',
    'turn right': 'RIGHT',
    'go right': 'RIGHT',
    
    'stop': 'STOP',
    'halt': 'STOP',
    'pause': 'STOP',
    
    'fog': 'FOG',
    'fogg': 'FOG',
    'find fog': 'FOG',
    'detect fog': 'FOG',
    
    'bag': 'BAG',
    'find bag': 'BAG',
    'detect bag': 'BAG',
    'search bag': 'BAG'
}

class AIEnhancedVoiceSystem:
    def __init__(self):
        # Initialize pygame for audio playback with higher quality
        pygame.mixer.init(frequency=22050, size=-16, channels=2, buffer=1024)
        
        # Speech recognition setup with improved settings
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 300  # Increased to reduce noise pickup
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.pause_threshold = 0.8   # Slightly longer pause
        self.recognizer.phrase_threshold = 0.3
        self.recognizer.non_speaking_duration = 0.4
        
        # Audio buffer with noise filtering
        buffer_size = int(SAMPLE_RATE * AUDIO_BUFFER_SECONDS)
        self.audio_buffer = deque(maxlen=buffer_size)
        self.buffer_lock = threading.Lock()
        
        # Processing queue
        self.audio_queue = queue.Queue(maxsize=2)
        
        # Counters
        self.packet_count = 0
        self.process_count = 0
        self.keyword_count = 0
        self.ai_response_count = 0
        
        # Audio playback control
        self.is_speaking = False
        self.speaking_lock = threading.Lock()
        
        # Sockets
        self.receive_socket = None
        self.send_socket = None
        self.audio_send_socket = None
        
        print("🚀 AI-Enhanced Voice System initialized with Gemini")

    def setup_sockets(self):
        """Setup UDP sockets for communication"""
        try:
            # Socket to receive audio data from ESP32
            self.receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.receive_socket.bind(('0.0.0.0', AUDIO_RECEIVE_PORT))
            self.receive_socket.settimeout(1.0)
            
            # Socket to send commands back to ESP32
            self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            # Socket to send audio responses to ESP32
            self.audio_send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            print(f"✅ Sockets setup complete")
            print(f"   📥 Listening for audio on port {AUDIO_RECEIVE_PORT}")
            print(f"   📤 Will send commands to {ESP32_IP}:{COMMAND_SEND_PORT}")
            print(f"   🔊 Will send audio to {ESP32_IP}:{AUDIO_SEND_PORT}")
            return True
            
        except Exception as e:
            print(f"❌ Socket setup failed: {e}")
            return False

    def apply_noise_reduction(self, audio_samples):
        """Apply noise reduction and filtering"""
        try:
            # Apply simple noise gate
            noise_gate_threshold = 100
            audio_samples = np.where(np.abs(audio_samples) < noise_gate_threshold, 0, audio_samples)
            
            # Apply simple high-pass filter to reduce low-frequency noise
            # This is a basic implementation - for better results, use scipy.signal
            if len(audio_samples) > 2:
                filtered = np.copy(audio_samples.astype(np.float32))
                alpha = 0.95  # High-pass filter coefficient
                for i in range(1, len(filtered)):
                    filtered[i] = alpha * (filtered[i-1] + filtered[i] - audio_samples[i-1])
                audio_samples = filtered.astype(np.int16)
            
            return audio_samples
        except Exception as e:
            print(f"❌ Noise reduction error: {e}")
            return audio_samples

    def convert_i2s_to_pcm_16bit(self, i2s_buffer):
        """Convert I2S 16-bit samples to PCM with noise reduction"""
        try:
            samples_16bit = np.frombuffer(i2s_buffer, dtype=np.int16)
            
            if len(samples_16bit) > 0:
                # Apply noise reduction
                samples_16bit = self.apply_noise_reduction(samples_16bit)
                
                if self.packet_count % 200 == 1:
                    max_amplitude = max(
                        abs(int(samples_16bit.min())),
                        abs(int(samples_16bit.max())),
                    )
                    print(f"🔍 Audio amplitude: {max_amplitude}")
            
            return samples_16bit
            
        except Exception as e:
            print(f"❌ I2S conversion error: {e}")
            return np.array([], dtype=np.int16)

    def create_wav_file(self, audio_samples):
        """Create temporary WAV file from 16-bit PCM samples with improved quality"""
        try:
            max_amplitude = (
                max(abs(int(audio_samples.min())), abs(int(audio_samples.max())))
                if len(audio_samples) > 0
                else 0
            )
            if max_amplitude < 150:  # Increased threshold for better signal detection
                return None
            
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                temp_path = temp_file.name
            
            # Normalize audio to improve recognition
            if max_amplitude > 0:
                normalized_samples = (audio_samples.astype(np.float32) / max_amplitude * 16000).astype(np.int16)
            else:
                normalized_samples = audio_samples.astype(np.int16)
            
            audio_bytes = normalized_samples.tobytes()
            
            with wave.open(temp_path, 'wb') as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(SAMPLE_RATE)
                wav_file.writeframes(audio_bytes)
            
            return temp_path
            
        except Exception as e:
            print(f"❌ WAV file creation error: {e}")
            return None

    def get_gemini_response(self, text):
        """Get AI response using Gemini API"""
        try:
            if not GEMINI_API_KEY:
                return "I heard you but I don't have an API key configured."
            
            headers = {
                'Content-Type': 'application/json'
            }
            
            payload = {
                "contents": [
                    {
                        "parts": [
                            {
                                "text": f"You are a helpful assistant in a voice-controlled robot. Keep responses short and conversational, under 40 words. If someone asks about movement, remind them of available commands: forward, backward, left, right, stop, fog detection, bag detection. User said: {text}"
                            }
                        ]
                    }
                ]
            }
            
            url = f"{GEMINI_URL}?key={GEMINI_API_KEY}"
            response = requests.post(url, headers=headers, json=payload, timeout=10)
            
            if response.status_code == 200:
                result = response.json()
                if 'candidates' in result and len(result['candidates']) > 0:
                    return result['candidates'][0]['content']['parts'][0]['text'].strip()
                else:
                    return "I heard you but couldn't generate a response."
            else:
                print(f"❌ Gemini API error: {response.status_code}")
                return f"I heard: {text}. Please give me a movement command."
                
        except Exception as e:
            print(f"❌ Gemini response error: {e}")
            return f"I heard you say: {text}. How can I help?"

    def text_to_speech(self, text):
        """Convert text to speech with improved audio quality"""
        temp_mp3_path = None
        try:
            with self.speaking_lock:
                self.is_speaking = True
            
            print(f"🔊 Converting to speech: '{text}'")
            
            # Create TTS audio with slower speech for clarity
            tts = gTTS(text=text, lang='en', slow=False, tld='com')
            
            # Save to temporary file
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as temp_mp3:
                temp_mp3_path = temp_mp3.name
            
            tts.save(temp_mp3_path)
            time.sleep(0.1)
            
            # Convert MP3 to high-quality PCM
            try:
                from pydub import AudioSegment
                from pydub.effects import normalize, compress_dynamic_range
                
                # Load and enhance audio
                audio = AudioSegment.from_mp3(temp_mp3_path)
                
                # Audio enhancements for clarity
                audio = normalize(audio)  # Normalize volume
                audio = compress_dynamic_range(audio)  # Compress dynamic range
                
                # Convert to optimal format for ESP32
                audio = audio.set_frame_rate(22050)
                audio = audio.set_channels(1)
                audio = audio.set_sample_width(2)  # 16-bit
                
                # Increase volume by 12dB for better audibility
                audio = audio + 12
                
                # Get raw PCM data
                pcm_data = audio.raw_data
                
                print(f"🔊 Enhanced audio: {len(pcm_data)} bytes, {len(audio)}ms, +12dB")
                
                # Play locally with higher volume
                try:
                    with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_wav:
                        temp_wav_path = temp_wav.name
                    
                    enhanced_audio = audio + 6  # Additional 6dB for local playback
                    enhanced_audio.export(temp_wav_path, format="wav")
                    
                    pygame.mixer.music.load(temp_wav_path)
                    pygame.mixer.music.set_volume(1.0)  # Maximum volume
                    pygame.mixer.music.play()
                    
                    while pygame.mixer.music.get_busy():
                        time.sleep(0.1)
                    
                    try:
                        os.unlink(temp_wav_path)
                    except:
                        pass
                        
                except Exception as play_error:
                    print(f"⚠️ Local playback error: {play_error}")
                
                # Send enhanced PCM data to ESP32
                self.send_pcm_to_esp32(pcm_data)
                
            except ImportError:
                print("⚠️ pydub not available, using basic method")
                try:
                    pygame.mixer.music.load(temp_mp3_path)
                    pygame.mixer.music.set_volume(1.0)
                    pygame.mixer.music.play()
                    while pygame.mixer.music.get_busy():
                        time.sleep(0.1)
                except Exception as fallback_error:
                    print(f"❌ Fallback playback error: {fallback_error}")
            
            return True
            
        except Exception as e:
            print(f"❌ Text-to-speech error: {e}")
            return False
        finally:
            # Clean up MP3 file
            if temp_mp3_path and os.path.exists(temp_mp3_path):
                try:
                    pygame.mixer.music.stop()
                    time.sleep(0.1)
                    os.unlink(temp_mp3_path)
                except Exception as cleanup_error:
                    print(f"⚠️ Cleanup warning: {cleanup_error}")
            
            with self.speaking_lock:
                self.is_speaking = False

    def send_pcm_to_esp32(self, pcm_data):
        """Send raw PCM data to ESP32 with improved transmission"""
        try:
            # Send start marker
            self.audio_send_socket.sendto(b'START_AUDIO', (ESP32_IP, AUDIO_SEND_PORT))
            time.sleep(0.1)
            
            # Send PCM data in optimized chunks
            chunk_size = 400  # Smaller chunks for better reliability
            total_chunks = len(pcm_data) // chunk_size + (1 if len(pcm_data) % chunk_size else 0)
            
            print(f"📤 Sending {len(pcm_data)} bytes in {total_chunks} chunks")
            
            for i in range(0, len(pcm_data), chunk_size):
                chunk = pcm_data[i:i + chunk_size]
                
                # Add chunk header with sequence number
                seq_num = i // chunk_size
                header = struct.pack('<HH', seq_num, len(chunk))
                packet = header + chunk
                
                self.audio_send_socket.sendto(packet, (ESP32_IP, AUDIO_SEND_PORT))
                time.sleep(0.015)  # Slightly faster transmission
                
                # Progress indicator
                if seq_num % 50 == 0:
                    progress = (seq_num + 1) * 100 // total_chunks
                    print(f"📤 Progress: {progress}%")
            
            time.sleep(0.1)
            # Send end marker
            self.audio_send_socket.sendto(b'END_AUDIO', (ESP32_IP, AUDIO_SEND_PORT))
            print(f"✅ Enhanced audio transmission complete")
            
        except Exception as e:
            print(f"❌ Failed to send PCM to ESP32: {e}")

    def detect_keywords_or_ai(self, audio_samples):
        """Detect keywords or process with AI"""
        wav_file_path = None
        try:
            wav_file_path = self.create_wav_file(audio_samples)
            if not wav_file_path:
                return None, None
            
            with sr.AudioFile(wav_file_path) as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=0.2)
                audio = self.recognizer.record(source)
            
            try:
                text = self.recognizer.recognize_google(audio, language='en-US').lower().strip()
                print(f"🎯 Recognized: '{text}'")
                
                # Check for exact keyword matches first
                if text in MOTOR_COMMANDS:
                    command = MOTOR_COMMANDS[text]
                    print(f"✅ KEYWORD MATCH: '{text}' -> '{command}'")
                    return command, None
                
                # Check for partial keyword matches
                for keyword, command in MOTOR_COMMANDS.items():
                    if keyword in text:
                        print(f"✅ PARTIAL KEYWORD: '{keyword}' found in '{text}' -> '{command}'")
                        return command, None
                
                # No keyword found - process with Gemini AI
                print(f"🤖 No keyword found, processing with Gemini AI: '{text}'")
                ai_response = self.get_gemini_response(text)
                return None, ai_response
                
            except sr.UnknownValueError:
                print("🔍 No speech detected")
                return None, None
            except sr.RequestError as e:
                print(f"❌ Speech recognition error: {e}")
                return None, None
            
        except Exception as e:
            print(f"❌ Detection error: {e}")
            return None, None
        finally:
            if wav_file_path and os.path.exists(wav_file_path):
                try:
                    os.unlink(wav_file_path)
                except:
                    pass

    def send_command_to_esp32(self, command):
        """Send command back to ESP32"""
        try:
            if self.send_socket:
                message = command.encode('utf-8')
                self.send_socket.sendto(message, (ESP32_IP, COMMAND_SEND_PORT))
                print(f"📤 Sent command to ESP32: '{command}'")
                return True
        except Exception as e:
            print(f"❌ Failed to send command to ESP32: {e}")
        return False

    def audio_receiver(self):
        """Receive I2S audio data from ESP32"""
        print(f"🎤 Audio receiver started - listening on port {AUDIO_RECEIVE_PORT}")
        
        last_stats_time = time.time()
        last_packet_time = time.time()
        
        while True:
            try:
                # Skip receiving audio if we're currently speaking
                with self.speaking_lock:
                    if self.is_speaking:
                        time.sleep(0.1)
                        continue
                
                data, addr = self.receive_socket.recvfrom(4096)
                current_time = time.time()
                
                if self.packet_count % 100 == 0:
                    time_since_last = current_time - last_packet_time
                    print(f"🔍 Packet {self.packet_count}: {len(data)} bytes, interval: {time_since_last:.3f}s")
                
                last_packet_time = current_time
                
                # i2s_read() can return a partial buffer. Accept every valid
                # 16-bit PCM datagram instead of dropping partial reads.
                if 0 < len(data) <= I2S_READ_LEN and len(data) % 2 == 0:
                    self.packet_count += 1
                    
                    pcm_samples = self.convert_i2s_to_pcm_16bit(data)
                    
                    if len(pcm_samples) > 0:
                        with self.buffer_lock:
                            self.audio_buffer.extend(pcm_samples)
                            
                            buffer_samples = len(self.audio_buffer)
                            required_samples = int(SAMPLE_RATE * PROCESS_CHUNK_SECONDS)
                            
                            if buffer_samples >= required_samples:
                                recent_samples = np.array(list(self.audio_buffer)[-required_samples:])
                                
                                if not self.audio_queue.full():
                                    try:
                                        self.audio_queue.put(recent_samples, block=False)
                                    except queue.Full:
                                        pass
                
                # Print stats every 10 seconds
                if current_time - last_stats_time >= 10.0:
                    buffer_seconds = len(self.audio_buffer) / SAMPLE_RATE
                    queue_size = self.audio_queue.qsize()
                    print(f"📊 Stats: Packets={self.packet_count}, Buffer={buffer_seconds:.1f}s, Queue={queue_size}")
                    print(f"   Keywords={self.keyword_count}, AI Responses={self.ai_response_count}")
                    print(f"   Speaking={self.is_speaking}")
                    last_stats_time = current_time
                
            except socket.timeout:
                current_time = time.time()
                with self.speaking_lock:
                    if not self.is_speaking and current_time - last_packet_time > 5.0:
                        print("⚠️ No packets received for 5+ seconds. Check ESP32 connection.")
                continue
            except Exception as e:
                print(f"❌ Audio receiver error: {e}")
                time.sleep(0.1)

    def audio_processor(self):
        """Process audio data for keyword detection or AI response"""
        print("🔍 Audio processor started")
        
        while True:
            try:
                # Skip processing if we're currently speaking
                with self.speaking_lock:
                    if self.is_speaking:
                        time.sleep(0.5)
                        continue
                
                audio_samples = self.audio_queue.get(timeout=3.0)
                self.process_count += 1
                
                print(f"🔍 Processing chunk {self.process_count}: {len(audio_samples)} samples")
                
                # Detect keywords or get AI response
                command, ai_text = self.detect_keywords_or_ai(audio_samples)
                
                if command:
                    # Keyword detected - send command to ESP32
                    self.keyword_count += 1
                    print(f"✅ KEYWORD #{self.keyword_count} DETECTED! Command: {command}")
                    
                    # Start the physical action before network TTS so a slow
                    # response service cannot delay the tracking trigger.
                    if self.send_command_to_esp32(command):
                        print(f"🚀 Command '{command}' sent to ESP32!")

                    if command == 'BAG':
                        print(f"🎒 BAG command detected - speaking confirmation")
                        self.text_to_speech("Okay, I'm searching for the bag")
                elif ai_text:
                    # No keyword - send AI response as speech
                    self.ai_response_count += 1
                    print(f"🤖 GEMINI RESPONSE #{self.ai_response_count}: {ai_text}")
                    
                    # Convert AI response to speech and send to ESP32
                    self.text_to_speech(ai_text)
                
                # Clear queue and buffer after processing
                cleared_items = 0
                while not self.audio_queue.empty():
                    try:
                        self.audio_queue.get_nowait()
                        cleared_items += 1
                    except queue.Empty:
                        break
                
                with self.buffer_lock:
                    self.audio_buffer.clear()
                
                if cleared_items > 0:
                    print(f"🔄 Cleared {cleared_items} queue items and buffer")
                
            except queue.Empty:
                print("🔍 Audio processor: No new audio to process...")
                continue
            except Exception as e:
                print(f"❌ Audio processor error: {e}")
                time.sleep(0.1)

    def run(self):
        """Start the AI-enhanced voice system"""
        print("🚀 Starting AI-Enhanced Voice Command System with Gemini")
        print("=" * 70)
        print(f"📡 ESP32 IP: {ESP32_IP}")
        print(f"💻 PC IP: {PC_IP}")
        print(f"🎯 Motor commands: {list(set(MOTOR_COMMANDS.values()))}")
        print(f"🤖 AI Engine: Google Gemini")
        print(f"🔊 Enhanced Audio: Noise reduction, volume boost, clarity improvements")
        print("-" * 70)
        
        if not self.setup_sockets():
            return
        
        # Start audio receiver thread
        receiver_thread = threading.Thread(target=self.audio_receiver, daemon=True)
        receiver_thread.start()
        print("✅ Audio receiver thread started")
        
        # Start audio processor thread  
        processor_thread = threading.Thread(target=self.audio_processor, daemon=True)
        processor_thread.start()
        print("✅ Audio processor thread started")
        
        print("\n🎤 System ready for voice commands and conversations!")
        print("💡 Enhanced Features:")
        print("   - Motor commands: forward, backward, left, right, stop, fog, bag")
        print("   - Gemini AI conversations: Ask questions, get spoken responses")
        print("   - High-quality TTS: Enhanced audio with noise reduction")
        print("   - Smart audio control: No listening during speech output")
        print("\nPress Ctrl+C to stop")
        print("=" * 70)
        
        try:
            while True:
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\n🛑 Stopping AI-enhanced voice system...")
        finally:
            if self.receive_socket:
                self.receive_socket.close()
            if self.send_socket:
                self.send_socket.close()
            if self.audio_send_socket:
                self.audio_send_socket.close()
            pygame.mixer.quit()

def main():
    print("🤖 ESP32-S3 AI-Enhanced Voice System with Gemini")
    print("=" * 60)
    
    # Check dependencies
    missing_deps = []
    try:
        import numpy as np
        print("✅ NumPy available")
    except ImportError:
        missing_deps.append("numpy")
    
    try:
        import speech_recognition as sr
        print("✅ SpeechRecognition available") 
    except ImportError:
        missing_deps.append("SpeechRecognition")
    
    try:
        import requests
        print("✅ Requests available")
    except ImportError:
        missing_deps.append("requests")
    
    try:
        from gtts import gTTS
        print("✅ gTTS available")
    except ImportError:
        missing_deps.append("gtts")
    
    try:
        import pygame
        print("✅ Pygame available")
    except ImportError:
        missing_deps.append("pygame")
    
    try:
        from pydub import AudioSegment
        print("✅ Pydub available - Enhanced audio processing enabled")
    except ImportError:
        print("⚠️ Pydub not available - install with: pip install pydub")
        print("   Audio will use basic processing")
    
    if missing_deps:
        print(f"❌ Missing dependencies: {', '.join(missing_deps)}")
        print("Install with: pip install " + " ".join(missing_deps))
        print("Also install: pip install pydub")
        return
    
    print(f"📡 Configuration:")
    print(f"   ESP32 IP: {ESP32_IP}")
    print(f"   Audio Port: {AUDIO_RECEIVE_PORT}")
    print(f"   Command Port: {COMMAND_SEND_PORT}")
    print(f"   Audio Send Port: {AUDIO_SEND_PORT}")
    print(f"   AI Engine: {GEMINI_MODEL}")
    print()
    
    # Create and run system
    system = AIEnhancedVoiceSystem()
    system.run()

if __name__ == "__main__":
    main()
