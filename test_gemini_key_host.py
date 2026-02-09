#!/usr/bin/env python3
"""
Test Gemini API key - can run on host (no container needed)
"""

import os
import sys

api_key = sys.argv[1] if len(sys.argv) > 1 else os.getenv("GOOGLE_API_KEY")

if not api_key:
    print("Usage: python3 test_gemini_key_host.py [API_KEY]")
    print("   Or: export GOOGLE_API_KEY='your-key' && python3 test_gemini_key_host.py")
    sys.exit(1)

print(f"Testing API key: {api_key[:10]}...{api_key[-4:]}\n")

try:
    # Try to import - if not available, give helpful error
    try:
        from langchain_google_genai import ChatGoogleGenerativeAI
    except ImportError:
        print("Error: langchain_google_genai not installed on host")
        print("\nTo test in container instead:")
        print("  1. Start simulation: ./run_simulation.sh")
        print("  2. Enter container: sudo docker exec -it <CONTAINER_ID> /bin/bash")
        print("  3. Run: source /opt/rosa-venv/bin/activate")
        print("  4. Run: python3 /root/test_gemini_key.py 'YOUR_KEY'")
        sys.exit(1)
    
    llm = ChatGoogleGenerativeAI(
        model="gemini-2.5-flash",
        temperature=0,
        google_api_key=api_key
    )
    
    print("Making API call...")
    response = llm.invoke("Say 'API key works' if you can read this.")
    
    print("\n" + "="*70)
    print("✅ SUCCESS! API KEY IS VALID AND WORKING!")
    print("="*70)
    print(f"Response: {response.content}\n")
    
except Exception as e:
    error_str = str(e)
    print("\n" + "="*70)
    print("✗ ERROR")
    print("="*70)
    print(f"{error_str}\n")
    
    if "API key" in error_str.lower() or "401" in error_str or "403" in error_str:
        print("❌ API KEY IS INVALID OR HAS NO PERMISSIONS")
    elif "429" in error_str:
        print("⚠️  RATE LIMITED - Key works but hit limit")
    else:
        print("⚠️  Check error message above")

