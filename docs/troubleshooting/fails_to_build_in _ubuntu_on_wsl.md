# Troubleshooting Fails  to Build in Ubuntu on WSL

1. Error getting credentials

## 1.  Error getting credentials
ERROR: failed to build: failed to solve: error getting credentials - err: fork/exec /usr/bin/docker-credential-desktop.exe: exec format error, out: `

### 1.1. What the error really means

This line is the key:
```text
error getting credentials - err: fork/exec /usr/bin/docker-credential-desktop.exe: exec format error
```

Docker is trying to run:
```text
/usr/bin/docker-credential-desktop.exe
```

But:

That’s a Windows binary (`.exe`),

You’re in Linux (Ubuntu in WSL),

So Linux says: “I can’t execute this file format” → `exec format error`.

This is a known issue when Docker Desktop is integrated with WSL but the credential helper is misconfigured. 
Tori's Repositories
+1

You’re just trying to pull a public image (`osrf/ros:humble-desktop-full`), so you don’t even need any credentials here.

### 1.2. Fix: `edit ~/.docker/config.json` in Ubuntu

Inside your Ubuntu-22.04 WSL (the shell where you ran `docker build`):

Show your Docker config:
```bash
cat ~/.docker/config.json
```

You’ll probably see something like:
```json
{
  "credsStore": "desktop.exe"
}
```

Edit the file:
```bash
nano ~/.docker/config.json
```

And change it to either of these:

Option A – simplest (no credential store at all):
```json
{}
```

Option B – “disable” it by renaming the key (trick from several WSL users): 
Tori's Repositories
```json
{
  "credStore": "desktop.exe"
}
```

Note the missing s in `credStore` – Docker ignores it.

Save & exit:

In `nano`:

`Ctrl+X` (exit) + follow dialog  to save the file

(Optional, but clean) Restart WSL from PowerShell:
```powershell 
wsl --shutdown
```

Then reopen Ubuntu 22.04 from the Start menu and go back to your repo:
```bash
cd ~/llm-pick-me-bots
```
3. Retry the build

Now in Ubuntu-22.04:
```
docker build --no-cache -t limo-cobot-humble .
```
Or use filepath for example
```
docker build --no-cache -t limo-cobot-humble -f ./Dockerfile.humble .
```

This time Docker should:

Skip the broken docker-credential-desktop.exe helper.

Pull osrf/ros:humble-desktop-full directly from Docker Hub.

Continue with the rest of the Dockerfile.

If it fails again, paste the new error log (it’ll likely be a different issue, like some ROS/CMake thing, which we can then tackle next).