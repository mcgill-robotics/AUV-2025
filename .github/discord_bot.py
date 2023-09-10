# bot.py
import os

import discord
import argparse
import json

# arg parse issue assignees
parser = argparse.ArgumentParser()
parser.add_argument("assignees",type=list)
args = parser.parse_args()
assignees = args.assignees
assignees = json.loads("{ \"assignees\":" + assignees + "}")


names = [assignee['login'] for assignee in assignees["assignees"]]


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
                    await channel.send("Test, an issue has been closed, assingees data is " + str(assignees))



client.run(TOKEN)