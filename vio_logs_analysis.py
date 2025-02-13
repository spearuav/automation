import re
import os
from datetime import datetime, timedelta

LOG_FILES = {
    "Mavlink": "mavlink.log",
    "VINS": "vins.log",
    "VIO": "vio.log"
}

ERROR_PATTERNS = [
    "error", "failed", "not found", "timeout", "crash", "exception", "segmentation fault"
]

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
    for service, log_file in LOG_FILES.items():
        if os.path.exists(log_file):
            report += f"Analyzing {service} Log ({log_file}):\n"
            issues, timestamp_issues = analyze_log(log_file)
            
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
                
            report += "\n"
        else:
            report += f"Log file {log_file} not found.\n"
    
    with open("log_analysis_report.txt", "w", encoding='utf-8') as report_file:
        report_file.write(report)
    
    print("Analysis completed. Report saved as log_analysis_report.txt")

if __name__ == "__main__":
    main()
