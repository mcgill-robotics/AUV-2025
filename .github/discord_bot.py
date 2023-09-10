# bot.py
import os

import discord
import argparse
import json

# arg parse issue assignees
parser = argparse.ArgumentParser()
parser.add_argument("num")
args = parser.parse_args()
num = args.num

num = str(num)

# names = [assignee['login'] for assignee in assignees["assignees"]]


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
                if channel.name == "discord-support":
                    await channel.send("Test, an issue has been closed, issue number: " + num)



client.run(TOKEN)