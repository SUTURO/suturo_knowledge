# suturo_knowledge

## Wichtige Desingentscheidungen und Prolog patterns:
1. Bei `tell(is_at(name,...))` wird vom Namen am Anfang Zahlen und Minusse entfernt. Damit weniger Bugs auftreten, sind die Knowledge Objektklassen angepasst und store_object_info_server.py passt die Namen entsprechend an.  
   Zum einfacheren Umgang entfernt Knowledge zusätzlich alle Unterstriche und macht den ersten Buchstaben groß.  
   Also `re.sub(r"^[0-9-_]+|_","",name).capitalize()` in Python.  
2. ifs in Prolog sehen so aus: `condition -> if_true; if_false`.  
   Das beudeutet aber, das wenn man loggen will, wenn ein teil fehlschlägt brauch man folgendes Konstrukt:  
   `(<query> -> true; ros_info("<query> failed"))`
