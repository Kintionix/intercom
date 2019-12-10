# Implementing a Data-Flow Control algorithm.

import sounddevice as sd
import numpy as np
import struct

from intercom_binaural import Intercom_binaural

if __debug__:
    import sys

class Intercom_dfc(Intercom_binaural):

    def init(self, args):
        Intercom_binaural.init(self, args)
        self.packet_format = f"!HBB{self.frames_per_chunk//8}B"
        self.saved = [0]*self.cells_in_buffer #64 de tamaño
        self.chunkmemory = [0]*self.cells_in_buffer
        self.flow = self.number_of_channels*16 #numero de bitplanes a enviar




    def record_send_and_play_stereo(self, indata, outdata, frames, time, status):
        indata[:,0] -= indata[:,1]
        self.record_and_send(indata)
        self._buffer[self.played_chunk_number % self.cells_in_buffer][:,0] += self._buffer[self.played_chunk_number % self.cells_in_buffer][:,1]
        
        chunk = self._buffer[self.played_chunk_number % self.cells_in_buffer]
        chunk[:, 0] += chunk[:, 1]
        self.play(outdata , chunk)

    def record_and_send(self, indata):
        print("played chunk", self.played_chunk_number)
       
        if(self.saved[(self.played_chunk_number+1) % self.cells_in_buffer] >= 4 and self.saved[self.played_chunk_number % self.cells_in_buffer] < self.flow):
            self.flow = self.saved[(self.played_chunk_number+1) % self.cells_in_buffer]

        print("bitplane played: ", self.saved[(self.played_chunk_number+1) % self.cells_in_buffer], "flow: ", self.flow)

        self.saved[self.played_chunk_number % self.cells_in_buffer] = 0
        self.chunkmemory[self.played_chunk_number%self.cells_in_buffer] = 0
        #self.saved[self.played_chunk_number] = self.played_chunk_number
        #print("Lista", *self.saved)
        for bitplane_number in range(self.number_of_channels*16-1, -1, -1):
            #print("bitplane number: ", bitplane_number, "flow : ", self.flow)
            if(bitplane_number >= self.flow):
                break
            #self.saved[self.played_chunk_number] = bitplane_number
            #print("Lista", *self.saved)
            #cabecera = [self.recorded_chunk_number][bitplane_number]
            #: devuelve todo el contenido de ese lado del array. >> desplaza los bits recogidos de ese array tantas casillas como avanza la i 
            # en el bucle local. & 1 hace que cuando un bit es negativo, el desplazamiento no arrastre mas 1 creados por la propiedad de desplazar los bits de un numero negativo
            bitplane = (indata[:, bitplane_number%self.number_of_channels] >> bitplane_number//self.number_of_channels) & 1
            #Convertimos el array a unsigned int 8
            bitplane = bitplane.astype(np.uint8)
            #Empaquetamos el array
            bitplane = np.packbits(bitplane)
            #Le damos forma al paquete que vamos a enviar con su formato de paquete, numero de chunk y todo el array del bitplane.
            message = struct.pack(self.packet_format, self.recorded_chunk_number, bitplane_number, *bitplane)
            #Enviamos el mensaje con su formato
            self.sending_sock.sendto(message, (self.destination_IP_addr, self.destination_port))
        #Cuando se han enviado todos los planos de bits, esta variable aumenta en 1 para hacer un seguimiento del contador de chunks enviados.
        self.recorded_chunk_number = (self.recorded_chunk_number + 1) % self.MAX_CHUNK_NUMBER


    def receive_and_buffer(self):
        message, source_address = self.receiving_sock.recvfrom(Intercom_binaural.MAX_MESSAGE_SIZE)
        #Despieza el paquete recibido en varias variables
        chunk_number, bitplane_number, *bitplane = struct.unpack(self.packet_format, message)
        #Crea una variable y le da formato para hacer legible el array recibido
        bitplane = np.asarray(bitplane, dtype=np.uint8)
        #Desempaqueta los bits en el formato inicial de indata
        bitplane = np.unpackbits(bitplane)
        #Transforma el array a in16 para hacer legible el nuevo array
        bitplane = bitplane.astype(np.int16)
        #Almacena en el chunk correspondiente el plano de bits, |= es un operador logico (OR) con el mismo funcionamiento que +=
        #necesario porque el buffer almacena los datos en un array bidimensional, si asignamos sin el operador el array que nos llega
        # del bitplane, sobreescribiriamos los valores de las columnas que no nos interesan tomar del array
        
        self.saved[chunk_number%self.cells_in_buffer] += 1
        #if(self.chunkmemory[chunk_number%self.cells_in_buffer] == 1):
         #   self.chunkmemory[chunk_number%self.cells_in_buffer] = 0
        if(self.saved[chunk_number%self.cells_in_buffer] == self.flow and self.flow < 32 and self.chunkmemory[chunk_number%self.cells_in_buffer] == 0):
            #print("+1")
            self.chunkmemory[chunk_number%self.cells_in_buffer] = 1
            self.flow += 1
            #self.saved[chunk_number%self.cells_in_buffer] = 0
        #print("El minimo actual es ", self.min)
        self._buffer[chunk_number % self.cells_in_buffer][:, bitplane_number%self.number_of_channels] |= (bitplane << bitplane_number//self.number_of_channels)
        #self.chunkmemory[chunk_number%self.cells_in_buffer] = chunk_number
        
        print("Lista", *self.saved)
        return chunk_number

    def play(self, outdata):
        #print("play ", outdata)
        chunk = self._buffer[self.played_chunk_number % self.cells_in_buffer]
        print("save: ", self.saved[(self.played_chunk_number+1) % self.cells_in_buffer])
        if(self.saved[(self.played_chunk_number+1) % self.cells_in_buffer] == self.flow and self.flow < 32):
            self.flow += 1
        self._buffer[self.played_chunk_number % self.cells_in_buffer] = self.generate_zero_chunk()
        self.played_chunk_number = (self.played_chunk_number + 1) % self.cells_in_buffer
        chunk = self.sm2tc(chunk)

        outdata[:] = chunk
        if __debug__:
            sys.stderr.write("."); sys.stderr.flush()
    
    def sm2tc(self, x):
        m = x >> 15
        return (~m & x) | (((x & 0x8000) - x) & m).astype(np.int16)

    def tc2sm(self, x):
        return ((x & 0x8000) | abs(x)).astype(np.int16)

if __name__ == "__main__":
    intercom = Intercom_dfc()
    parser = intercom.add_args()
    args = parser.parse_args()
    intercom.init(args)
    intercom.run()
