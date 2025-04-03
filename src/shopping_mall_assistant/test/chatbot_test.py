from deepeval.test_case import LLMTestCase, ConversationalTestCase
from deepeval.metrics import AnswerRelevancyMetric, FaithfulnessMetric, ConversationRelevancyMetric
from langchain_openai import AzureChatOpenAI
from deepeval.models.base_model import DeepEvalBaseLLM
from openai import AzureOpenAI
import os
import json

# Imposta una variabile di ambiente
os.environ["OPENAI_API_KEY"] = "8WKEvna2S3GkUq1NRWYQGY5CU3fxTrANLGIDRsgdSLJx688fUvK4JQQJ99BAAC5T7U2XJ3w3AAABACOGb9rz"

class AzureModel(DeepEvalBaseLLM):
    def __init__(
        self,
        model
    ):
        self.model = model

    def load_model(self):
        return self.model

    def generate(self, prompt: str) -> str:
        chat_model = self.load_model()
        return chat_model.invoke(prompt).content

    async def a_generate(self, prompt: str) -> str:
        chat_model = self.load_model()
        res = await chat_model.ainvoke(prompt)
        return res.content

    def get_model_name(self):
        return "Custom Azure OpenAI Model"

instructions = """You are Pepper, a friendly robotic assistant in the shopping centre. You speak naturally but accurately. When you greet someone you show that you're accomodating. If they ask you to check the file, in the answer you cannot mention the files, only answer the rest of the request.

                TONE AND STYLE:
                - Use informal but informative language.
                - Maintain a conversational tone (spoken conversation style).
                - Connect sentences smoothly.
                - Avoid at any cost lists and numbering. Write a smooth and well formed period instead.
                - Avoid opening phrases like "the following", "here is", or "I'm providing".

                CODE INTERPRETER:
                - shopping_centre.json contains all the information about the people in the shopping centre, consult it if you get a question about that. L'id è assistant-hTCY0nXQE1Wmx1GFfNqiDcFw 
                - contest.json contains all the information about the contest's participants and results, consult it if you get a question about that. L'id è assistant-VkIipH9BFp6gStJLtGsNRjRD 
                - always double check your answers based on the files
                - only open the file containing the information you need. You can understand it from the questions.

                STRUCTURE OF SHOPPING_CENTRE INFORMATION:
                - The field "gender" indicates the person's gender. The value is "male" for men and "female" for women.
                - The field "hat" indicates whether the person is wearing a hat or not. The value is "true" if the person is wearing a hat, otherwise "false".
                - The field "bag" indicates whether the person is carrying a bag or not. The value is "true" if the person is carrying a bag, otherwise "false".
                - The field "bar_passages", "cinema_passages", "supermarket_passages", and "bookshop_passages" indicate how many times the person has passed by the respective location. If the value is 0, the person has not passed by that place.


                WHEN SEARCHING FOR A PERSON:
                - Pay attention to the sentence to identify the gender based on pronouns, adjectives, articles, etc. For example, “my nephew” implies male, as "my" refers to masculine in this context.

                COUNTING PEOPLE IN THE SHOPPING CENTRE (WITHOUT SPECIFICS):
                - Simply state the total number of people and how many are men and how many are women.
                - Do not mention attributes, locations, or anything like that.

                WHEN DESCRIBING PEOPLE:
                - Always start by communicating the total number.
                - Then, for EACH person described:
                * Specify the gender (man/woman).
                * Do not mention the person's ID.
                * Specify ALL the locations where the person has been seen.
                * Do NOT mention how many times they've been seen at each location.
                * Do NOT mention attributes not requested in the question.
                * Do NOT refer to what the person does NOT have/wear.
                * Ensure descriptions are smooth and conversational, as if spoken.

                FOR THE CONTEST:
                - Start the sentence naturally (e.g., “In first place, we have...”).
                - Do NOT use "group zero" but instead "group one", "group two", etc.
                - Do NOT use lists or bullet points.
                - Do NOT say "the following" or "here is".
                - Answer questions directly without adding unrequested information.
                - Do NOT list the names of group members unless explicitly requested.
                - If asked for the winner, top three, or something similar, only state the group number. Do not mention the members or the score.
                - Do not provide the score as a percentage but in decimal form.
                - When given the name of a contestant check for mispellings to find matches, for example cristian could be christian or prugno-siniscalchi could be prugno siniscalchi.
                - Sometimes you will receive only the first or the second name of a contestant, so take this into account
                - Not all groups have the same number of members

                RESPONSE STRUCTURE:
                1. Total number (for people).
                2. Smooth and continuous description.
                3. Only requested information.
                4. Do NOT make up locations, attributes, or anything of the sort.
                5. RESPOND IN BRITISH ENGLISH.
                6. DO NOT WRITE EMOTICONS.
                7. IT IS FORBIDDEN TO ANSWER WITH LISTS LIKE BULLET LISTS, THE ANSWER MUST BE LIKE A SPOKEN CONVERSATION, A SPEECH
"""

custom_model = AzureChatOpenAI(
    openai_api_version="2024-08-01-preview",
    azure_deployment="gpt-4o",
    azure_endpoint="https://pepperoni.openai.azure.com/",
    openai_api_key="8WKEvna2S3GkUq1NRWYQGY5CU3fxTrANLGIDRsgdSLJx688fUvK4JQQJ99BAAC5T7U2XJ3w3AAABACOGb9rz",
)

azure_openai = AzureModel(model=custom_model)

# Configura il client Azure OpenAI
client = AzureOpenAI(
    api_key="8WKEvna2S3GkUq1NRWYQGY5CU3fxTrANLGIDRsgdSLJx688fUvK4JQQJ99BAAC5T7U2XJ3w3AAABACOGb9rz",
    azure_endpoint="https://pepperoni.openai.azure.com/",
    api_version="2024-08-01-preview"
)

assistant = client.beta.assistants.retrieve(assistant_id="asst_ERa6QRKuc1eVSNOh9ODXKfQe")

# ID dell'assistente precedentemente creato
assistant_id = assistant.id

# Crea un thread per la conversazione
thread = client.beta.threads.create()

# Aggiunge il messaggio dell'utente al thread
client.beta.threads.messages.create(
    thread_id=thread.id,
    role="user",
    content="tell me which files were uploaded and their data structure"
)

# Esegue l'assistant sul thread
run = client.beta.threads.runs.create(
    thread_id=thread.id,
    assistant_id=assistant_id
)

# Attende il completamento del run
while run.status != "completed":
    run = client.beta.threads.runs.retrieve(
        thread_id=thread.id,
        run_id=run.id
    )
        
# Funzione per ottenere la risposta dall'assistant
def get_assistant_response(input_text):

    # Aggiunge il messaggio dell'utente al thread
    client.beta.threads.messages.create(
        thread_id=thread.id,
        role="user",
        content=input_text + " (check the file carefully)"
    )

    # Esegue l'assistant sul thread
    run = client.beta.threads.runs.create(
        thread_id=thread.id,
        assistant_id=assistant_id
    )

    # Attende il completamento del run
    while run.status != "completed":
        run = client.beta.threads.runs.retrieve(
            thread_id=thread.id,
            run_id=run.id
        )

    # Recupera i messaggi dal thread
    messages = client.beta.threads.messages.list(thread_id=thread.id)
    response = messages.data[0].content[0].text.value  # La risposta dell'assistant

    return response

# Carica i test case dal file JSON
with open("/TestSet/test_cases.json", "r", encoding="utf-8") as f:
    test_cases_data = json.load(f)  # deve essere una lista di dizionari con 'input' e 'retrieval_context'

# Definisce le metriche
relevancy_metric = AnswerRelevancyMetric(include_reason=True, model=azure_openai)
faithfulness_metric = FaithfulnessMetric(include_reason=True, model=azure_openai)

# Per la valutazione conversazionale
conv_metric = ConversationRelevancyMetric(model=azure_openai)

# Liste per salvare i punteggi di ogni singolo test case
relevancy_scores = []
faithfulness_scores = []

test_cases = []

for idx, pair in enumerate(test_cases_data):
    input_text = pair["input"]
    retrieval_context = pair["retrieval_context"]

    actual_output = get_assistant_response(input_text)

    # Crea il test case
    single_case = LLMTestCase(
        input=input_text,
        actual_output=actual_output,
        retrieval_context=[retrieval_context]
    )

    # Misura le metriche
    relevancy_metric.measure(test_case=single_case)
    faithfulness_metric.measure(test_case=single_case)

    # Salva i punteggi calcolati
    relevancy_scores.append(relevancy_metric.score)
    faithfulness_scores.append(faithfulness_metric.score)

    print(f"Test Case {idx}:")
    print(f"Input: {input_text}")
    print(f"Actual Output: {actual_output}")
    print(f"Retrieval Context: {retrieval_context}")
    print(f"Relevance Score: {relevancy_metric.score}")
    print(f"Faithfulness Score: {faithfulness_metric.score}")
    print("=" * 50)

    test_cases.append(single_case)

# Calcola le medie delle metriche
avg_relevancy = sum(relevancy_scores) / len(relevancy_scores) if len(relevancy_scores) > 0 else 0
avg_faithfulness = sum(faithfulness_scores) / len(faithfulness_scores) if len(faithfulness_scores) > 0 else 0

print(f"Average Relevance Score (all test cases): {avg_relevancy}")
print(f"Average Faithfulness Score (all test cases): {avg_faithfulness}")

# Valutazione conversazionale
conv_case = ConversationalTestCase(turns=test_cases)
conv_metric.measure(conv_case)
print(f'Conversation Score: {conv_metric.score}')