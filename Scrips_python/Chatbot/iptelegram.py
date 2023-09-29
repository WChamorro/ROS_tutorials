#!/usr/bin/python3
import os

import telebot
import subprocess


BOT_TOKEN = 'your_token_here'

bot = telebot.TeleBot(BOT_TOKEN)


@bot.message_handler(commands=['start', 'hello'])
def send_welcome(message):
    msg = subprocess.check_output("hostname -I", shell=True).decode('ascii')
    bot.reply_to(message,msg)

bot.infinity_polling()

