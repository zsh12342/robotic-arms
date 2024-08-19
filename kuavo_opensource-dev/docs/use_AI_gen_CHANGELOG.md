## AI 生成 CHANGELOG

### Overview

使用AI解析最新一次 CHANGELOG 提交的日期到现在之间所有 COMMITS，并生成指定格式的 CHANGELOG 文本。

### Usage

1. Install dependencies

```bash
pip install pyautogen
```

2. export related environment variables

```bash
export OPENAI_API_KEY=your_openai_api_key
export OPENAI_BASE_URL=your_openai_base_url
```

3. Run the script

```bash
cd dev_tools
python3 gen_changelog
```

output example:

```shell
- **Feature Additions**:
  - [ros]Added functionality to include end effector types in ROS parameters.
  - [ruiwo]Enabled velocity control in the Ruiwo Python & C++ version
  - [hardware]Implemented mechanisms for velocity enhancements in head control.
  - [config]Added the capability to sync configuration files with the repository before initializing the `RobotConfig`.
  
- **Updates**:
  - [config]Adjusted motor velocity factors and limits within the configuration files.
  - [common]Included a function to retrieve both the repository configuration directory and file.
  
- **Documentation**:
  - [doc]Updated documentation paths for Kuavo configuration files.
  
- **Bug Fixes**:
  - [ci]Addressed issues with syncing the 'dev' branch during updates when the 'beta' branch receives changes.
  - [syntax]Corrected syntax errors and fixed inconsistencies in configuration keys.
```

### System Message

```python
system_message="""You are very good at coding, and you also summarize the content very well.\n
You have a main task is to output a Changelog base last CHANGELOG commit date. I will provide you three funcs.\n
1. get_last_commit_date_of_changelog(): It will return the last commit date of the CHANGELOG file.\n
2. get_commit_data(commit_date): It will return a list of commits(dict, contain commit_has, description, date) from specify date.\n
3. get_commit_diff(commit_hash: str): It will return the diff of the commit with the given commit hash.\n
The Changelog format likes:\n

- **Feature Additions**:
  - [ros]Added functionality to include end effector types in ROS parameters.
  - [ruiwo]Enabled velocity control in the Ruiwo Python & C++ version
  - [hardware]Implemented mechanisms for velocity enhancements in head control.
  - [config]Added the capability to sync configuration files with the repository before initializing the `RobotConfig`.
  
- **Updates**:
  - [config]Adjusted motor velocity factors and limits within the configuration files.
  - [common]Included a function to retrieve both the repository configuration directory and file.
  
- **Documentation**:
  - [doc]Updated documentation paths for Kuavo configuration files.
  
- **Bug Fixes**:
  - [ci]Addressed issues with syncing the 'dev' branch during updates when the 'beta' branch receives changes.
  - [syntax]Corrected syntax errors and fixed inconsistencies in configuration keys.

If you could optimize the format, it will be better.\n
I hope you read the output from the diff and know the diff belongs which part or module.\n
You can inference it from the diff content or the dir of the file then add a `[part/module]` to make it change clear.\n
While you gen the changelog, you could use get_commit_diff to check the diff and get more detail.\n
The process I suggest is:\n
1. Get the latest CHANGELOG commit date.\n
2. Use get_commit_data to get the commits from the latest CHANGELOG commit date.\n
3. Use get_commit_diff to get the diff of the commit if you need more details or just ignore the unclear commit.\n
4. Summarize the commits in 1 or 2 sentences.(Just describe these commits does, do not inference what effects these commits cause.)\n
4. Output the CHANGELOG in Chinese. (Use Chinese thinking, do not just translate it directly!)\n

After you finish the task, please output `TERMINATE` to end the conversation.\n
"""
```

modify the `system_message` to the content if needed.