import os
import json
import requests
import msal

# Load secrets from environment variables
CLIENT_ID = os.getenv("OAUTH_CLIENT_ID")#"8da4e345-cb32-43b6-a809-accd6ad28b96"#os.getenv("OAUTH_CLIENT_ID")
CLIENT_SECRET = os.getenv("OAUTH_CLIENT_SECRET")#"hIn8Q~ezVFTXzAoP54FZqHSHNjTr_-w5Gr_kpcPp"#os.getenv("OAUTH_CLIENT_SECRET")
TENANT_ID = os.getenv("OAUTH_TENANT_ID")#"53040ea5-7f4b-4d9d-99e2-659e24e12639"#os.getenv("OAUTH_TENANT_ID")

EMAIL_SENDER = "eitan.amit@spearuav.com"
EMAIL_RECIPIENT = "eitan.amit@spearuav.com"
EMAIL_SUBJECT = "Email test 2"
EMAIL_BODY = "This is a test email sent using Microsoft Graph API instead of SMTP."

# Microsoft Graph API endpoints
AUTHORITY = f"https://login.microsoftonline.com/{TENANT_ID}"
TOKEN_URL = f"{AUTHORITY}/oauth2/v2.0/token"
GRAPH_API_URL = "https://graph.microsoft.com/v1.0/users/{}/sendMail".format(EMAIL_SENDER)

# Get OAuth2 token
app = msal.ConfidentialClientApplication(CLIENT_ID, CLIENT_SECRET, authority=AUTHORITY)
token = app.acquire_token_for_client(scopes=["https://graph.microsoft.com/.default"])

if "access_token" not in token:
    print("❌ Token Response Error:", token)  # Print full error response
    raise Exception("Failed to obtain access token")

access_token = token["access_token"]
print("✅ Access Token Obtained Successfully")

# Create email payload
email_payload = {
    "message": {
        "subject": EMAIL_SUBJECT,
        "body": {
            "contentType": "Text",
            "content": EMAIL_BODY
        },
        "toRecipients": [
            {"emailAddress": {"address": EMAIL_RECIPIENT}}
        ]
    }
}

# Send email using Microsoft Graph API
headers = {
    "Authorization": f"Bearer {access_token}",
    "Content-Type": "application/json"
}

response = requests.post(GRAPH_API_URL, headers=headers, data=json.dumps(email_payload))

if response.status_code == 202:
    print("✅ Email sent successfully!")
else:
    print(f"❌ Failed to send email. Response: {response.status_code}, {response.text}")
