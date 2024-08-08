from ollama import Client

class LLM:
    def __init__(self, host, role, model) -> None:
        self.host = host
        self.role = role
        self.model = model

    @property
    def get_host(self):
        return self.host
    
    @property
    def get_role(self):
        return self.role


    def prompt(self, content):

        client = Client(host=self.host)
        response = client.chat(model=self.model, messages=[
        {
            'role': self.role,
            'content': content,
        },
        ])

        return(response['message']['content'])