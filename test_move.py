import requests

url = "http://localhost:4060/move"

# Define the payload with the coordinates
payload = {
    "x": 0.2,
    "y": -0.2,
    "z": 0.2
}

# Send the POST request
response = requests.post(url, json=payload)

# Print the response
print(response.json())

# Code after the response
print("This code is executed immediately after sending the request.")