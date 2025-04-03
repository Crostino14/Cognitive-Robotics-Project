#!/usr/bin/env python3
from shopping_mall_assistant.srv import Dialogue, DialogueResponse

import rospy
import requests
from openai import AzureOpenAI
import re
import time
import os

class DialogueServer():
    """
    A ROS (Robot Operating System) node for handling dialogue interactions using Azure OpenAI services.

    This node manages user queries, processes them through a pre-configured AI assistant, and returns natural language responses.

    Attributes:
    - client (AzureOpenAI): Client for interacting with Azure OpenAI services.
    - chatty_assis (AzureOpenAI.Assistant): Pre-configured assistant for managing conversations.

    Methods:
    - __init__(self): Initializes the node, loads resources, and configures the AI assistant.
    - _handle_service(self, req): Handles incoming dialogue requests and returns AI-generated responses.
    - start(self): Starts the ROS node and spins to handle requests continuously.
    """

    def __init__(self) -> None:
        """
        Initializes the DialogueServer class, setting up the Azure OpenAI client, and loading necessary resources.
        """
        rospy.init_node('dialogue service')
        rospy.Service('dialogue_server', Dialogue, self._handle_service)
        
        self.client = AzureOpenAI(
            api_key="YOUR OPENAI KEY",
            azure_endpoint="YOUR AZURE OPENAI ENDPOINT",
            api_version="2024-08-01-preview"
        )
    
        self.chatty_assis = self.client.beta.assistants.retrieve(assistant_id="YOUR ASSISTANT ID")
        
        self.thread = self.client.beta.threads.create()
        
        message = self.client.beta.threads.messages.create(
            thread_id=self.thread.id,
            role='user',
            content="tell me which files were uploaded, their data structure and all the information about a single element of both files"
        )
        
        run = self.client.beta.threads.runs.create(
            assistant_id=self.chatty_assis.id,
            thread_id=self.thread.id
        )
        
        while run.status in ['queued', 'in_progress', 'cancelling']:
            time.sleep(1)
            run = self.client.beta.threads.runs.retrieve(
                thread_id=self.thread.id,
                run_id=run.id
            )
        
        if run.status == 'completed':
            messages = self.client.beta.threads.messages.list(
                thread_id=self.thread.id
            )
            
            latest_message = next(msg for msg in messages if msg.role == 'assistant')
            cleaned_message = re.sub(r"【.*?】", "", latest_message.content[0].text.value)
            
            print(cleaned_message)
            
    
    def _handle_service(self, req):
        """
        Handles incoming dialogue requests, sending them to the assistant and returning the AI-generated response.

        Args:
        - req (Dialogue): The service request containing the user input text.

        Returns:
        - DialogueResponse: The AI-generated response.
        """
        input_text = req.input_text
        input_text += " (check the file)"

        message = self.client.beta.threads.messages.create(
            thread_id=self.thread.id,
            role='user',
            content=input_text
        )

        run = self.client.beta.threads.runs.create(
            assistant_id=self.chatty_assis.id,
            thread_id=self.thread.id
        )

        while run.status in ['queued', 'in_progress', 'cancelling']:
            time.sleep(1)
            run = self.client.beta.threads.runs.retrieve(
                thread_id=self.thread.id,
                run_id=run.id
            )

        cleaned_message = ""
        
        if run.status == 'completed':
            messages = self.client.beta.threads.messages.list(
                thread_id=self.thread.id
            )

            latest_message = next(msg for msg in messages if msg.role == 'assistant')
            cleaned_message = re.sub(r"【.*?】", "", latest_message.content[0].text.value)
        else:
            print(f"\nError: {run.status}")

        response = DialogueResponse()
        response.answer = cleaned_message

        return response

    def start(self):
        """
        Starts the ROS node and spins to handle dialogue requests.
        """
        rospy.logdebug('Dialogue server READY.')
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

if __name__ == '__main__':
    ds = DialogueServer()
    ds.start()
