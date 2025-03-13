import os
import requests
import sys
import re

API_KEY = 'eyJhbGciOiJIUzI1NiJ9.eyJ0aWQiOjQ2MjE2NTA3NCwiYWFpIjoxMSwidWlkIjo3MTA0MDcxMSwiaWFkIjoiMjAyNS0wMS0yM1QxNToxMjoyMy4wMDBaIiwicGVyIjoibWU6d3JpdGUiLCJhY3RpZCI6MTExMjcwMjUsInJnbiI6InVzZTEifQ.FpAB0SDf2FhzZq2DOz8YM05ne1C-UOAu278uplnk4MQ'
TEMPLATE_BOARD_ID = 8687569486

def duplicate_board(new_board_name):
    headers = {
        'Authorization': API_KEY,
        'Content-Type': 'application/json'
    }
    query = '''
    mutation {
      duplicate_board(
        board_id: %d,
        duplicate_type: duplicate_board_with_pulses,
        board_name: "%s"
      ) {
        board { id }
      }
    }
    ''' % (TEMPLATE_BOARD_ID, new_board_name)

    response = requests.post('https://api.monday.com/v2', json={'query': query}, headers=headers)

    if response.status_code == 200 and 'errors' not in response.json():
        print("Board duplicated successfully.")
    else:
        print("Failed:", response.text)
        sys.exit(1)

if __name__ == "__main__":
    filename = os.getenv("FILENAME")
    match = re.search(r'viper300-\d+\.\d+-(\d+\.\d+\.\d+)\.json', filename)
    if match:
        version = match.group(1)
        new_board_name = f"Release {version}"
        duplicate_board(new_board_name)
    else:
        print("Filename doesn't match pattern")
        sys.exit(0)
