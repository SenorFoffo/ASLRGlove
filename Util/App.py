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
	def updateApp(self, csocket, inputticker = []):
		#if sys.stdin.read(1) == "close\n":
			#print "Closing app."
			#csocket.close()
			#sys.exit()
		data = csocket.recv(1)

		print data
	# if data != None:
	# 	letter = ''
	# 	print data
			# for l in self.knownchars:
			# 	if l in data and l != self.currentLetter:
		if data != None:
			self.updateCanvas(data, inputticker)
			# 		break
		self.root.after(500, self.updateApp, csocket, inputticker)


	def updateCanvas(self, d, itick):
		#l = re.sub("\n", "", letter)

		self.canvas.delete("all")
		if d != "\n":
			itick.append(d)
		self.printLetterlist(itick, d)
		self.canvas.update()
		pass

	def printLetterlist(self, itick, let):

		if let in self.knownchars:
			self.currentLetter = let
		self.canvas.create_text(250, 250, anchor="center", text=self.currentLetter, font=("Serif", 80))

		size = 20
		# first = True
		posx = 20
		posy = 30

		if len(itick) > 16:
			itick.pop(0)

		for letter in itick:
			self.canvas.create_text(posx, posy, anchor="center", text=letter, font=("Serif", size))
			posx += 30

def main():
	app = App()
	app.startGui()
	return 0

if __name__=="__main__":
	main()
