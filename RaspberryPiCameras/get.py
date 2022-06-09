import requests

print("calling...")

r = requests.get('http://172.16.1.201:9097')

print("text:", r.text)

payload="226 143 25 25 1"

r = requests.post('http://172.16.1.201:9097', data=payload)

print("text:", r.text)
