# bot.py
import os
import sys

import discord
import argparse
import requests

parser = argparse.ArgumentParser()
parser.add_argument("issue_number")
args = parser.parse_args()
num = int(args.issue_number)

response = requests.get(f"https://api.github.com/repos/mcgill-robotics/AUV-2024/issues/{num}")
json = response.json()
title = json["title"]
names = [assignee['login'] for assignee in json["assignees"]]

names_string = names[0]
for name in names[1:-1]:
    names_string += ", " + name
if len(names) > 1:
    names_string += " and " + names[-1]


message = "Hey Clarke here. Big thanks to " + names_string \
    + " for closing issue " + str(json["number"]) + " - " + title + ". I really appreciate your hard work!"


TOKEN = os.getenv("DISCORD_TOKEN")

intents = discord.Intents.default()
client = discord.Client(intents=intents)


@client.event
async def on_ready():
    print(f'{client.user} has connected to Discord!')
    await send_message()


async def send_message():
    for guild in client.guilds:
        if guild.name == "McGill Robotics":
            for channel in guild.channels:
                if channel.name == "auv-general":
                    await channel.send(message)
                    sys.exit(0)



client.run("MTE1MDE2MjAwMDc3MzY1MjU0MA.GOmtM7.bvwnfETCD7ZhIHQa0SH579h0opKQH6S_hypp6A")