#!/usr/bin/env python3
"""
Simple test to verify Gemini API key is working

Usage:
    python3 test_gemini_key.py [API_KEY]
    # Or
    export GOOGLE_API_KEY='your-key'
    python3 test_gemini_key.py
"""

import os
import sys

# Get API key from argument or environment variable
api_key = None
if len(sys.argv) > 1:
    api_key = sys.argv[1]
else:
    api_key = os.getenv("GOOGLE_API_KEY")

if not api_key:
    print("Error: No API key provided")
    print("\nUsage:")
    print("  python3 test_gemini_key.py [API_KEY]")
    print("  # Or")
    print("  export GOOGLE_API_KEY='your-key'")
    print("  python3 test_gemini_key.py")
    sys.exit(1)

print("="*70)
print("Gemini API Key Test")
print("="*70)
print(f"API Key: {api_key[:10]}...{api_key[-4:]}")
print()

try:
    print("Step 1: Importing langchain_google_genai...")
    from langchain_google_genai import ChatGoogleGenerativeAI
    print("✓ Import successful")
    print()
    
    print("Step 2: Creating LLM instance...")
    llm = ChatGoogleGenerativeAI(
        model="gemini-2.5-flash",
        temperature=0,
        google_api_key=api_key
    )
    print("✓ LLM instance created")
    print()
    
    print("Step 3: Making test API call...")
    print("  (This may take a few seconds...)")
    response = llm.invoke("Say 'API key is working' if you can read this.")
    print()
    
    print("="*70)
    print("✅ SUCCESS! API KEY IS VALID AND WORKING!")
    print("="*70)
    print(f"Response: {response.content}")
    print()
    print("Your Gemini API key is correctly configured and can make API calls.")
    
except ImportError as e:
    print("✗ Import Error:")
    print(f"  {e}")
    print()
    print("Make sure you're in the ROSA virtual environment:")
    print("  source /opt/rosa-venv/bin/activate")
    sys.exit(1)
    
except Exception as e:
    error_str = str(e)
    print("="*70)
    print("✗ ERROR OCCURRED")
    print("="*70)
    print(f"Error: {error_str}")
    print()
    
    # Check for specific error types
    if "API key" in error_str.lower() or "401" in error_str or "403" in error_str:
        print("❌ API KEY IS INVALID OR HAS NO PERMISSIONS")
        print()
        print("Possible issues:")
        print("  - Key is incorrect or expired")
        print("  - Key doesn't have Gemini API access enabled")
        print("  - Key has been revoked")
        print()
        print("Check your Google Cloud Console:")
        print("  https://console.cloud.google.com/apis/credentials")
    elif "429" in error_str:
        print("⚠️  API KEY IS VALID BUT RATE LIMITED")
        print()
        print("The key works, but you've hit the rate limit.")
        print("Wait a few minutes and try again.")
    elif "400" in error_str:
        print("⚠️  BAD REQUEST")
        print()
        print("Possible issues:")
        print("  - API endpoint issue")
        print("  - Model name issue")
        print("  - Request format problem")
    else:
        print("⚠️  UNKNOWN ERROR")
        print()
        print("Check the error message above for details.")
    
    sys.exit(1)

