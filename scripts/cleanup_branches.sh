#!/bin/bash

ORG="spearuav"
CUTOFF_DAYS=14

# Microsoft Graph API credentials
EMAIL_FROM="eitan.amit@spearuav.com"
EMAIL_ADMIN="eitan.amit@spearuav.com"

TOKEN_URL="https://login.microsoftonline.com/$TENANT_ID/oauth2/v2.0/token"

# Get access token
ACCESS_TOKEN=$(curl -s -X POST $TOKEN_URL \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -d "grant_type=client_credentials&client_id=$CLIENT_ID&client_secret=$CLIENT_SECRET&scope=https://graph.microsoft.com/.default" \
  | jq -r '.access_token')

if [[ -z "$ACCESS_TOKEN" || "$ACCESS_TOKEN" == "null" ]]; then
  echo "❌ Error: Failed to obtain access token."
  exit 1
fi

CUTOFF_DATE=$(date -d "$CUTOFF_DAYS days ago" +"%Y-%m-%d")

# Temporary file cleanup
> branch_cleanup_summary.csv
> branches_to_ignore.csv
> debug.txt

# Branches to ignore
IGNORED_BRANCHES=()

# Lists for action
NOTIFY_LIST=("eitan.amit@spearuav.com","ben.tubul@spearuav.com","chen.arazi@spearuav.com","eden@spearuav.com","eitan.amit@spearuav.com","noa.malul@spearuav.com","roie.geron@spearuav.com","tzuriel.lampner@spearuav.com","zafrir.zinger@spearuav.com","gil.cohn@spearuav.com","mandel.yonatan@gmail.com","noam.levy@spearuav.com","yair.shlomi@spearuav.com","guy.shadmon@spearuav.com")

echo "Checking for branches older than $CUTOFF_DAYS days..."

# Header for CSV file
echo "Repository,Branch,Last Commit Date,Lifespan,Owner Email,Action" > branch_cleanup_summary.csv

declare -A NOTIFY_EMAILS


# Iterate over all repositories
for repo in $(gh repo list $ORG --json name,defaultBranchRef --limit 200 --jq '.[] | @base64'); do
    repo_data=$(echo $repo | base64 --decode)
    repo_name=$(echo $repo_data | jq -r '.name')
    default_branch=$(echo $repo_data | jq -r '.defaultBranchRef.name')

    echo "Checking repository: $repo_name"
    echo "#################### Checking repository: $repo_name ################## " >> debug.txt

    # Iterate over branches
    for branch in $(gh api repos/$ORG/$repo_name/branches --jq '.[].name' || echo "error"); do
        echo >> debug.txt
        echo >> debug.txt
        echo >> debug.txt
        echo >> debug.txt
        echo "******* branch: $branch ************" >> debug.txt
        if [[ $branch == "error" ]]; then
			      echo "error!" >> debug.txt
            continue
        fi

        # Skip ignored branches
        if [[ "$branch" == "$default_branch" || " ${IGNORED_BRANCHES[@]} " =~ " $branch " ]]; then
            echo "$repo_name,$branch,IGNORED" >> branches_to_ignore.csv
            echo "skip" >> debug.txt
            echo "$repo_name,$branch,IGNORED" >> debug.txt
            continue
        fi

        # Get first commit info

        first_commit_sha=$(gh api repos/$ORG/$repo_name/branches/$branch --jq '.commit.sha' || echo "error")

        echo "first_commit_sha" >> debug.txt
        echo "=================" >> debug.txt
        echo "$first_commit_sha" >> debug.txt
        first_commit_info=$(gh api repos/$ORG/$repo_name/git/commits/$first_commit_sha || echo "error")
        echo "first_commit_info" >> debug.txt
        echo "=================" >> debug.txt
        echo "$first_commit_info" >> debug.txt
        branch_created_date=$(echo "$first_commit_info" | jq -r '.author.date' || echo "error")

        echo "branch_created_date" >> debug.txt
        echo "=================" >> debug.txt
        echo "$branch_created_date" >> debug.txt



        branch_created_date=$(echo "$first_commit_info" | jq -r '.author.date' || echo "error")
        echo "branch_created_date" >> debug.txt
        echo "=================" >> debug.txt
        echo "$branch_created_date" >> debug.txt
        branch_owner_email=$(echo "$first_commit_info" | jq -r '.author.email' || echo "unknown@example.com")
        echo "branch_owner_email" >> debug.txt
        echo "=================" >> debug.txt
        echo "$branch_owner_email" >> debug.txt


        if [[ "$branch_created_date" == "error" || "$last_commit_date" == "error" ]]; then
            continue
        fi

        # Format dates
        branch_created_date=$(date -d "$branch_created_date" +"%Y-%m-%d")
        last_commit_date=$(date -d "$last_commit_date" +"%Y-%m-%d")
        echo "branch_created_date" >> debug.txt
        echo "=================" >> debug.txt
        echo "$branch_created_date" >> debug.txt
        echo "last_commit_date" >> debug.txt
        echo "=================" >> debug.txt
        echo "$last_commit_date" >> debug.txt

        # Calculate lifespan
        branch_created_epoch=$(date -d "$branch_created_date" +%s)
        echo "branch_created_epoch" >> debug.txt
        echo "=================" >> debug.txt
        echo "$branch_created_epoch"	 >> debug.txt


        lifespan_days=$(( ( $(date +%s) - branch_created_epoch ) / 86400 ))
        echo "lifespan_days" >> debug.txt
        echo "=================" >> debug.txt
        echo "$lifespan_days"	 >> debug.txt

        # Check if branch is older than the cutoff
        if [[ "$branch_created_date" < "$CUTOFF_DATE" ]]; then
          if [[ " ${NOTIFY_LIST[*]} " =~ " ${branch_owner_email} " ]]; then
            echo "$repo_name,$branch,$branch_created_date,$lifespan_days,$branch_owner_email,NOTIFIED" >> branch_cleanup_summary.csv
            NOTIFY_EMAILS[$branch_owner_email]+="$repo_name,$branch,$branch_created_date,$lifespan_days\n"
          else
            echo "$repo_name,$branch,$branch_created_date,$lifespan_days,$branch_owner_email,IGNORED" >> branch_cleanup_summary.csv
          fi
        fi
    done
done

# Send grouped emails
for email in "${!NOTIFY_EMAILS[@]}"; do
    if [[ -z "$email" || "$email" == "null" ]]; then
        echo "⚠️ Skipping email: Invalid recipient"
        continue
    fi

    # Create a CSV file for each user
    USER_CSV="user_${email//[@.]/_}.csv"
    echo "Repo Name,Branch Name,Last Commit Date,Lifespan (days)" > "$USER_CSV"
    echo -e "${NOTIFY_EMAILS[$email]}" | tr '|' ',' >> "$USER_CSV"

    # Convert CSV to Base64 for attachment
    BASE64_USER_CSV=$(base64 -w 0 "$USER_CSV")

    # Create JSON payload
    JSON_FILE="email_payload_${email//[@.]/_}.json"

cat <<EOF > "$JSON_FILE"
{
    "message": {
        "subject": "[Branch Cleanup] Old Branches in $ORG",
        "body": {
            "contentType": "Text",
            "content": "Dear User,\n\nPlease find attached list of old branches in $ORG that are older than 14 days.\nPlease review and consider merging or deleting them.\n\nThanks,\nRepo Cleanup Bot"
        },
        "toRecipients": [
            { "emailAddress": { "address": "$email" } }
        ],
        "attachments": [
            {
                "@odata.type": "#microsoft.graph.fileAttachment",
                "name": "old_branches.csv",
                "contentBytes": "$BASE64_USER_CSV",
                "contentType": "text/csv"
            }
        ]
    }
}
EOF

    # Send the email with the attachment
    RESPONSE=$(curl -s -X POST "https://graph.microsoft.com/v1.0/users/$EMAIL_FROM/sendMail" \
      -H "Authorization: Bearer $ACCESS_TOKEN" \
      -H "Content-Type: application/json" \
      --data "@$JSON_FILE")

    # Cleanup temp files
    rm -f "$USER_CSV" "$JSON_FILE"

    # Check for errors
    if echo "$RESPONSE" | grep -q "error"; then
        echo "❌ Error sending email to $email: $RESPONSE"
    else
        echo "📩 Email with CSV attachment sent successfully to $email!"
    fi

    sleep 1  # Prevent rate limits

done

# Send summary email to admin
# File to attach
ATTACHMENT_PATH="branch_cleanup_summary.csv"
ATTACHMENT_NAME=$(basename "$ATTACHMENT_PATH")

# Convert the CSV file to Base64 safely
BASE64_CONTENT=$(base64 -w 0 "$ATTACHMENT_PATH" | tr -d '\n')

# Create JSON payload and save it to a temporary file
JSON_FILE="email_payload.json"

cat <<EOF > "$JSON_FILE"
{
    "message": {
        "subject": "[Branch Cleanup] Weekly Summary",
        "body": {
            "contentType": "Text",
            "content": "Attached is the weekly branch cleanup summary."
        },
        "toRecipients": [
            { "emailAddress": { "address": "$EMAIL_ADMIN" } }
        ],
        "attachments": [
            {
                "@odata.type": "#microsoft.graph.fileAttachment",
                "name": "$ATTACHMENT_NAME",
                "contentBytes": "$BASE64_CONTENT",
                "contentType": "text/csv"
            }
        ]
    }
}
EOF

# Send the email
RESPONSE=$(curl -s -X POST "https://graph.microsoft.com/v1.0/users/$EMAIL_FROM/sendMail" \
  -H "Authorization: Bearer $ACCESS_TOKEN" \
  -H "Content-Type: application/json" \
  --data "@$JSON_FILE")

# Cleanup temp file
rm -f "$JSON_FILE"

# Check for errors
if echo "$RESPONSE" | grep -q "error"; then
    echo "❌ Error sending email: $RESPONSE"
else
    echo "📩 Email with attachment sent successfully!"
fi

echo "✅ Cleanup Completed!"
