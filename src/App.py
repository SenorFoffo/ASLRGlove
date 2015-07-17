#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  App.py
#
#  Copyright 2015 Christoph Schulthess
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#
#
from Tkinter import *
from socket import *
import sys,re

class App:

	def __init__(self, title="ASLR Recognition", bg="black", w=500, h=500, host="", port=9999):

		#---known letters and letter list---#
		self.knownchars = ["A", "B", "D", "I", "L", "_"]
		self.letterlist = []
		self.currentLetter = "_"

		#---Init Tk gui, change parameters in __init__---#
		self.root = Tk()
		self.root.title(title)
		self.root.configure(background=bg)

		self.canvas = Canvas(self.root, width=w, height=h)
		self.canvas.pack(side=LEFT)

		#---init communication---#
		self.host = host
		self.port = port

		try:
			self.serversock = socket(AF_INET, SOCK_STREAM)
			self.serversock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
			self.serversock.bind((self.host, self.port))
			self.serversock.listen(1)
		except:
			print "Communication init failed."

	#---start update loop after device is connected---#
	def startGui(self):
		while 1:
			print "Waiting for connection... listening on port", self.port
			clientsocket, addr = self.serversock.accept()
			print "...connected from:", addr
			print "Starting App..."
			break;
		self.root.after(500, self.updateApp, clientsocket)
		self.root.mainloop()

	#---run update---#
	def updateApp(self, csocket):
		#if sys.stdin.read(1) == "close\n":
			#print "Closing app."
			#csocket.close()
			#sys.exit()
		data = csocket.recv(1024)
		print
		if data != None:
			letter = ''
			for l in self.knownchars:
				if l in data and l != self.currentLetter:
					self.updateCanvas(l)
					break
		self.root.after(500, self.updateApp, csocket)


	def updateCanvas(self, letter):
		#l = re.sub("\n", "", letter)
		self.currentLetter = letter
		self.canvas.delete("all")
		self.letterlist.append(letter)
		if len(self.letterlist) > 7:
			self.letterlist.pop(0)
		self.printLetterlist()
		print letter
		self.canvas.update()

	def printLetterlist(self):
		size = 40
		first = True
		posx = 400
		posy = 230

		for letter in reversed(self.letterlist):
			self.canvas.create_text(posx, posy, anchor="center", text=letter, font=("Serif", size))
			if first:
				first = False
				size = 20
			posx -= 50

def main():
	app = App()
	app.startGui()
	return 0

if __name__=="__main__":
	main()
