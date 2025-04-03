#!/usr/bin/python3
import rospy
from std_msgs.msg import String
from shopping_mall_assistant.srv import AnimatedSpeechService, AnimatedSpeechServiceRequest

def test_animated_speech_service():
    '''
    Script di test per il servizio "animated_speech_service".
    Invia un messaggio di testo al nodo AnimatedSpeechNode
    e verifica se il robot risponde correttamente con animazioni sincronizzate.
    '''
    rospy.init_node('animated_speech_test_node', anonymous=True)

    # Attendi che il servizio sia disponibile
    rospy.logdebug("Aspetto il servizio 'animated_speech_service'...")
    rospy.wait_for_service('animated_speech_service')

    try:
        # Connetti al servizio
        animated_speech_service = rospy.ServiceProxy('animated_speech_service', AnimatedSpeechService)
        
        # Crea una richiesta con un esempio di testo da pronunciare
        test_text = "Hello! How are you today?"
        request = AnimatedSpeechServiceRequest()
        request.input.data = test_text

        rospy.logdebug(f"Inviando testo al servizio: {test_text}")
        # Invia la richiesta
        response = animated_speech_service(request)

        # Stampa la risposta
        rospy.loginfo(f"Risposta dal servizio: {response.output.data}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Errore durante la chiamata al servizio: {e}")

if __name__ == "__main__":
    try:
        test_animated_speech_service()
    except rospy.ROSInterruptException:
        pass
