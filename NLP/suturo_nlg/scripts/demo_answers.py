import simplenlg
from simplenlg.lexicon import *
from simplenlg.framework import *
from simplenlg.realiser.english import *
from simplenlg.phrasespec import *
from simplenlg.features import *
import random
import time
import zmq
from tinyrpc.server import RPCServer
from tinyrpc.dispatch import RPCDispatcher
from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.zmq import ZmqServerTransport

ctx = zmq.Context()
dispatcher = RPCDispatcher()
transport = ZmqServerTransport.create(ctx, 'tcp://127.0.0.1:5002')

rpc_server = RPCServer(
    transport,
    JSONRPCProtocol(),
    dispatcher
)

lexicon = Lexicon().getDefaultLexicon()
nlgFactory = NLGFactory(lexicon)
realiser = Realiser(lexicon)

def randomString(strings):
    return strings[random.randint(0, len(strings) - 1)]

def describeTask(task, tense, progressive=True):
    p = nlgFactory.createClause()
    p.setFeature("tense", tense)
    p.setSubject("I")
    p.setVerb(task["action"])
    if "patient" in task:
        p.setObject(task["patient"])
        p.getObject().setDeterminer("the")
        patientCount = 1
        if "patientNumber" in task:
            patientCount = task["patientNumber"]
        if 0 == patientCount:
            p.getObject().setDeterminer("any")
            p.setFeature(Feature.NEGATED, True)
        p.getObject().setPlural((1 < patientCount))
    p.setFeature(Feature.PROGRESSIVE, progressive)
    if "destination" in task:
        place = nlgFactory.createNounPhrase(task["destination"])
        place.setDeterminer("the")
        pp = nlgFactory.createPrepositionPhrase()
        pp.addComplement(place)
        pp.setPreposition("to")
        p.addComplement(pp)
    return realiser.realiseSentence(p)

questions = ("Is there [qitem] on the [furniture_piece]?", "What is on the [furniture_piece]?", "Where is the [item] supposed to go?")
input_qitem = ("a red Pringles can ", "a blue Pringles can ", "a cup ", "some instant coffee powder ", "a banana ")
input_item = ("red Pringles can ", "blue Pringles can ", "cup ", "instant coffee powder ", "banana ")
input_furniture_piece = ("big table ", "small table ", "shelf ")

def knowledge_result_1():
    if knowledge == True:
        return ("Yes") #, there is [input_qitem] on the [input_furniture_piece]
    if knowledge == False:
        return ("No") #, there is no [input_qitem] on the [input_furniture_piece]

def knowledge_result_2():
    if knowledge == True:
        return "There is [input_qitem] on the [input_furniture_piece]"
    if knowledge == False:
        return "There is nothing on the [input_furniture_piece]"

#def DemoAnswers():
#    if question "Is there [input_qitem] on the [input_furniture_piece]?" == True:
 #       return knowledge_result_1
  #  if question "Is there [input_qitem] on the [input_furniture_piece]?" == False:
   #     return ""
    #if question "What is on the [input_furniture_piece]?" == True:
     #   return knowledge_result_2
    #if question "What is on the [input_furniture_piece]?" == False:
     #   return ""
    #if question "Where is the [input_item] supposed to go?" == True:
    #    return "The [input_item] is supposed to go on the [input_furniture_piece]"
    #if question "Where is the [input_item] supposed to go?" == False:
     #   return ""

@dispatcher.public
def generate_text(knowledge):
    if "question" in knowledge:
        return "huh?"
    query = knowledge["question"]
    if "is there" == query:
        if "yes" == knowledge["answer"]:
            return "yes"
	else: 
	    return "no" 
    elif query == "supposed to go":
	item = knowledge["item"]
	location = knowledge["location"]
        return "the %s is supposed to go on the %s"%(item, location)
    elif query == "what is on":
	item = knowledge["item"]
	location = knowledge["location"]
        return "there is a %s on the %s"%(item, location)
    return "huh?"
    
rpc_server.serve_forever()


