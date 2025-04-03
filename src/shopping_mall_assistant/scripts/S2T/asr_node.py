#!/usr/bin/python3
import numpy as np
import requests
import rospy
import openai
import requests
import wave
import os
from openai import AzureOpenAI
from std_msgs.msg import Int16MultiArray, String
from shopping_mall_assistant.srv import ASRService, ASRServiceResponse

# Azure OpenAI API Configuration
AZURE_OPENAI_ENDPOINT = "INSERT YOUR AZURE OPENAI ENDPOINT"
AZURE_OPENAI_API_KEY = "INSERT YOUR OPENAI KEY"
API_VERSION = "2024-06-01"

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUDIO_PATH = os.path.join(BASE_DIR, "/tmp/input_audio.wav")

class WhisperAzureSpeechRecognition:
    """
    A ROS (Robot Operating System) node for integrating Whisper via Azure AI.
    This node is responsible for converting speech to text using Whisper APIs provided by Azure OpenAI.

    Attributes:
    - sample_rate (int): The sampling rate for the audio in Hz. Default is 16000 Hz.
    - language_code (str): The language code for speech recognition. Default is 'en' (English).

    Methods:
    - __init__(self, sample_rate: int, language_code: str): Initializes the class with audio parameters.
    - __call__(self): Sets up the ROS service and starts the transcription node.
    - __handle_s2t(self, req: ASRService) -> ASRServiceResponse: Handles incoming audio requests and converts them to text.
    - transcribe_audio(self, file_path: str, deployment_name: str, api_key: str, endpoint: str, api_version: str) -> str:
      Sends the audio to the Whisper API for transcription and retrieves the response.
    """

    def __init__(self, sample_rate: int = 16000, language_code: str = 'en'):
        """
        Initializes the WhisperAzureSpeechRecognition class.

        Args:
            - sample_rate (int): The sampling rate of the audio data in Hz.
            - language_code (str): Language code for speech recognition (e.g., 'en' for English).
        """
        self.sample_rate = sample_rate
        self.language_code = language_code
        self.session = requests.Session()
        openai.api_key = AZURE_OPENAI_API_KEY
        openai.api_version = API_VERSION
        openai.azure_endpoint = AZURE_OPENAI_ENDPOINT
        openai.api_type = "azure"


    def perform_warmup_request(self):
        """
        Sends a warm-up request to the Azure Whisper API with a dummy audio file.
        """
        # Creare un piccolo file audio fittizio in memoria
        import io
        import wave

        rospy.logdebug("Creating dummy audio file for warm-up...")
        with wave.open(AUDIO_PATH, "wb") as audio_file:
            audio_file.setnchannels(1)  # Mono audio
            audio_file.setsampwidth(2)  # 16-bit audio
            audio_file.setframerate(self.sample_rate)  # 16 kHz sampling rate
            audio_file.writeframes(b'\x00' * self.sample_rate)  # 1 secondo di audio vuoto

        # Effettua la richiesta al modello Whisper
        url = f"{AZURE_OPENAI_ENDPOINT}/openai/deployments/Whisper-Model/audio/translations?api-version={API_VERSION}"
        headers = {"api-key": AZURE_OPENAI_API_KEY}

        with open(AUDIO_PATH, "rb") as audio_file:
            files = {"file": (AUDIO_PATH, audio_file, "audio/wav")}
            response = requests.post(url, headers=headers, files=files)

        if response.status_code == 200:
            rospy.loginfo("Warm-up request completed successfully.")
        else:
            rospy.logerr(f"Warm-up request failed: {response.status_code} - {response.text}")
            raise Exception(f"Warm-up failed with status {response.status_code}")

    def __call__(self):
        """
        Subscribes to the 'asr_service' and starts the ROS node for handling speech-to-text requests.
        """
        rospy.init_node('audio_speech_recognition', anonymous=True)
        rospy.Service('asr_service', ASRService, self.__handle_s2t)
        # Effettua una richiesta di warm-up
        try:
            rospy.loginfo("Performing warm-up request to Azure Whisper API...")
            self.perform_warmup_request()
            rospy.loginfo("Warm-up completed successfully.")
        except Exception as e:
            rospy.logwarn(f"Warm-up failed: {e}")
        rospy.spin()

    def __handle_s2t(self, req: ASRService) -> ASRServiceResponse:
        """
        Callback function for handling speech-to-text service requests. Converts audio data to text.

        Args:
            - req (ASRService): The service request containing audio data in Int16 format.

        Returns:
            - ASRServiceResponse: The service response containing the transcribed text.
        """
        try:
            rospy.logdebug("Audio received, processing transcription...")

            if not req.input.data:
                raise ValueError("No audio data received in the request.")

            # Convert audio data from Int16 to bytes
            audio_data = bytearray()
            for value in req.input.data:
                if -32768 <= value <= 32767:
                    audio_data.extend(value.to_bytes(2, byteorder='little', signed=True))
                else:
                    raise ValueError(f"Value out of range for Int16: {value}")

            # Save the audio data to a temporary WAV file
            with wave.open(AUDIO_PATH, "wb") as audio_file:
                audio_file.setnchannels(1)  # Mono audio
                audio_file.setsampwidth(2)  # 16-bit audio
                audio_file.setframerate(self.sample_rate)  # 16 kHz sampling rate
                audio_file.writeframes(audio_data)

            rospy.logdebug("Audio data saved, sending to Azure Whisper API...")

            # Perform transcription using the Whisper API on Azure
            transcript = self.transcribe_audio(
                file_path=AUDIO_PATH,
                deployment_name="Whisper-Model",
                api_key=AZURE_OPENAI_API_KEY,
                endpoint=AZURE_OPENAI_ENDPOINT,
                api_version=API_VERSION,
            )
            rospy.loginfo(f"Transcription completed: \n\n{transcript}\n\n")
            return ASRServiceResponse(output=String(data=transcript))

        except ValueError as ve:
            rospy.logerr(f"Error in request data: {ve}")
            return ASRServiceResponse(output=String(data="Error: invalid data"))
        except Exception as e:
            rospy.logerr(f"Error during transcription: {e}")
            return ASRServiceResponse(output=String(data="Error during transcription"))

    def transcribe_audio(self, file_path: str, deployment_name: str, api_key: str, endpoint: str, api_version: str) -> str:
        """
        Sends audio data to the Whisper API on Azure for transcription.

        Args:
            - file_path (str): Path to the audio file to be transcribed.
            - deployment_name (str): Deployment name of the Whisper model.
            - api_key (str): Azure OpenAI API key.
            - endpoint (str): Azure OpenAI endpoint URL.
            - api_version (str): API version to use for the request.

        Returns:
            - str: The transcribed text from the audio.

        Raises:
            - Exception: If the API call fails or returns an error.
        """
        url = f"{endpoint}/openai/deployments/{deployment_name}/audio/translations?api-version={api_version}"
        headers = {
            "api-key": api_key,
        }
        rospy.logdebug(f"Sending API request to {url}")

        with open(file_path, "rb") as audio_file:
            files = {"file": (file_path, audio_file, "audio/wav")}
            response = requests.post(url, headers=headers, files=files)
        
        if response.status_code == 200:
            rospy.loginfo("Transcription successfully completed by the API.")
            return response.json().get("text", "")
        else:
            error_message = f"API Error: {response.status_code} - {response.text}"
            rospy.logerr(error_message)
            raise Exception(error_message)

if __name__ == "__main__":
    recognizer = WhisperAzureSpeechRecognition()
    recognizer()
