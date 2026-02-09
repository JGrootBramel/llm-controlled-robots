#!/bin/bash
# Simple Gemini API key test using curl (no Python needed)

API_KEY="${1:-${GOOGLE_API_KEY}}"

if [ -z "$API_KEY" ]; then
    echo "Usage: ./test_gemini_key_simple.sh [API_KEY]"
    echo "   Or: export GOOGLE_API_KEY='your-key' && ./test_gemini_key_simple.sh"
    exit 1
fi

echo "======================================================================"
echo "Testing Gemini API Key"
echo "======================================================================"
echo "API Key: ${API_KEY:0:10}...${API_KEY: -4}"
echo ""

# Test the API key with a simple request
RESPONSE=$(curl -s -w "\n%{http_code}" \
    -H "Content-Type: application/json" \
    -H "X-Goog-Api-Key: ${API_KEY}" \
    "https://generativelanguage.googleapis.com/v1beta/models/gemini-2.5-flash:generateContent" \
    -d '{
        "contents": [{
            "parts": [{
                "text": "Say hello if you can read this"
            }]
        }]
    }')

HTTP_CODE=$(echo "$RESPONSE" | tail -n1)
BODY=$(echo "$RESPONSE" | sed '$d')

echo "HTTP Status Code: $HTTP_CODE"
echo ""

if [ "$HTTP_CODE" = "200" ]; then
    echo "✅ SUCCESS! API KEY IS VALID AND WORKING!"
    echo ""
    echo "Response:"
    echo "$BODY" | grep -o '"text":"[^"]*"' | head -1 | sed 's/"text":"//;s/"$//'
    echo ""
elif [ "$HTTP_CODE" = "401" ] || [ "$HTTP_CODE" = "403" ]; then
    echo "❌ API KEY IS INVALID OR HAS NO PERMISSIONS"
    echo ""
    echo "Check your Google Cloud Console:"
    echo "  https://console.cloud.google.com/apis/credentials"
elif [ "$HTTP_CODE" = "429" ]; then
    echo "⚠️  RATE LIMITED - Key works but hit rate limit"
    echo "   Wait a few minutes and try again"
else
    echo "⚠️  ERROR (HTTP $HTTP_CODE)"
    echo ""
    echo "Response:"
    echo "$BODY" | head -20
fi

echo "======================================================================"

