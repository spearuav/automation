import re
import os
from datetime import datetime, timedelta

analysis_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

LOG_DIR = os.path.expanduser("~/automation-logs/")

LOG_FILES = {
    "Mavlink": os.path.join(LOG_DIR, "mavlink.log"),
    "VINS": os.path.join(LOG_DIR, "vins.log"),
    "VIO": os.path.join(LOG_DIR, "vio.log")
}

ERROR_PATTERNS = [
    "error", "failed", "not found", "timeout", "crash", "exception", "segmentation fault"
]

def get_log_timestamp(log_path):
    """Returns the last modified time of a log file in a human-readable format."""
    if os.path.exists(log_path):
        mod_time = os.path.getmtime(log_path)
        return datetime.fromtimestamp(mod_time).strftime("%Y-%m-%d %H:%M:%S")
    return "Log file not found"

def extract_timestamps(log_file):
    timestamps = []
    with open(log_file, 'r', encoding='utf-8') as file:
        for line in file:
            match = re.search(r'\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})\]', line)
            if match:
                timestamps.append(datetime.strptime(match.group(1), "%Y-%m-%d %H:%M:%S"))
    return timestamps

def check_timestamp_consistency(timestamps, threshold=5):
    inconsistencies = []
    for i in range(1, len(timestamps)):
        diff = (timestamps[i] - timestamps[i - 1]).total_seconds()
        if diff > threshold:
            inconsistencies.append((timestamps[i - 1], timestamps[i], diff))
    return inconsistencies

def analyze_log(log_file):
    issues = []
    timestamps = extract_timestamps(log_file)
    
    with open(log_file, 'r', encoding='utf-8') as file:
        for line in file:
            for pattern in ERROR_PATTERNS:
                if re.search(pattern, line, re.IGNORECASE):
                    issues.append(line.strip())
    
    timestamp_issues = check_timestamp_consistency(timestamps)
    
    return issues, timestamp_issues

def main():
    report = "Log Analysis Report\n\n"
    report += "=" * 50
    report += f"Log Analysis Created On: {analysis_time}"
    
    report += "\n**Log Files Information:**"

    for service, log_path in LOG_FILES.items():
        if os.path.exists(log_path):
            log_time = get_log_timestamp(log_path)
            report += f"Analyzing {service} Log ({log_path}):\n"
            report += f"Last Modified: {log_time}\n"
            issues, timestamp_issues = analyze_log(log_path)
            
            if issues:
                report += "- Errors & Failures Found:\n"
                for issue in issues[:10]:  # Show only first 10 for brevity
                    report += f"  {issue}\n"
                if len(issues) > 10:
                    report += "  ... more errors found\n"
            else:
                report += "- No critical errors found.\n"
                
            if timestamp_issues:
                report += "- Timestamp inconsistencies found:\n"
                for start, end, diff in timestamp_issues[:5]:  # Show only first 5
                    report += f"  Gap of {diff} seconds between {start} and {end}\n"
                if len(timestamp_issues) > 5:
                    report += "  ... more inconsistencies found\n"
            else:
                report += "- No timestamp inconsistencies detected.\n"
            
            report += "=" * 50 +"\n\n"
        else:
            report += f"Log file {log_path} not found.\n"
    
    report_path = os.path.join(LOG_DIR, "log_analysis_report.txt")
    with open(report_path, "w", encoding='utf-8') as report_file:
        report_file.write(report)
    
    print(f"Analysis completed. Report saved as: {report_path}")

if __name__ == "__main__":
    main()
