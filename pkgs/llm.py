from ollama import Client


class LLM:


    def __init__(self, host, role, model, instructions):
        self.host = host
        self.role = role
        self.model = model
        self.instructions = instructions

    @property
    def get_host(self):
        return self.host
    
    @property
    def get_role(self):
        return self.role


    def chat(self, content):

        client = Client(host=self.host)
        response = client.chat(model=self.model, messages=[
        {
            'role': self.role,
            'content': content,
        },
        ])

        print(response['message']['content'])

if __name__ == "__main__":
    newChat = LLM("192.168.2.120:11434", "user", "llama3", "You are a robot called QT")
    newChat.chat("why is the sky blue?")