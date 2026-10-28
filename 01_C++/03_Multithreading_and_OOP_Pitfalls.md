## Partie 1 : Les 8 Pièges du Multi-threading (QCM 1, 3 et 5)

### 1. Exception entre `lock()` et `unlock()` *(QCM 1)*

* **Le piège :** Appeler manuellement `mtx.lock()`, faire un traitement qui peut lever une exception (`throw`), puis appeler `mtx.unlock()`.
* **Pourquoi c'est un bug :** Si l'exception est déclenchée, l'exécution quitte immédiatement la fonction et saute la ligne `mtx.unlock()`. Le mutex reste verrouillé pour toujours, et tout autre thread qui tentera d'y accéder restera figé à l'infini (*deadlock*).
* **La solution :** Toujours utiliser le principe **RAII** avec `std::lock_guard<std::mutex>` ou `std::scoped_lock`. Lorsque la fonction s'arrête (normalement ou à cause d'une exception), l'objet `lock_guard` est détruit et son destructeur déverrouille automatiquement le mutex.

### 2. Lectures simultanées bloquées par `std::mutex` *(QCM 1)*

* **Le piège :** Croire que deux threads peuvent exécuter en parallèle une méthode `read()` qui ne fait que lire une variable, dès lors qu'elle utilise un `std::lock_guard<std::mutex>`.
* **Pourquoi c'est un bug :** Un `std::mutex` est un verrou strictement **exclusif**. Il ne fait aucune différence entre une lecture et une écriture : un seul thread peut posséder le verrou à un instant donné, obligeant les lecteurs à s'attendre les uns les autres.
* **La solution :** Utiliser `std::shared_mutex` associé à `std::shared_lock` pour les lectures (plusieurs threads peuvent lire en même temps) et `std::unique_lock` pour l'écriture (exclusivité totale).

### 3. Verrouiller un mutex dans une méthode `const` *(QCM 1)*

* **Le piège :** Déclarer une méthode de lecture `double read() const` qui verrouille le mutex `this->mtx` de la classe.
* **Pourquoi c'est un bug :** Cela provoque une **erreur de compilation**. Dans une méthode `const`, tous les attributs de l'objet deviennent constants. Or, verrouiller et déverrouiller un mutex modifie son état interne.
* **La solution :** Ajouter le mot-clé **`mutable`** devant la déclaration du mutex (`mutable std::mutex mtx;`), ce qui autorise sa modification même à l'intérieur d'une méthode `const`.

### 4. Ordre d'acquisition des verrous et Deadlock croisé *(QCM 3)*

* **Le piège :** Une fonction verrouille `s1.mtx` puis `s2.mtx`. Le Thread A appelle `fuse(imu, gps)` pendant que le Thread B appelle `fuse(gps, imu)` en inversant l'ordre des arguments.
* **Pourquoi c'est un bug :** C'est une **attente circulaire (*deadlock*)**. Le Thread A verrouille `imu` et attend `gps`, tandis que le Thread B a déjà verrouillé `gps` et attend `imu`. En revanche, si les deux threads appellent la fonction dans le **même ordre** `(imu, gps)`, il n'y a aucun deadlock : le second attend simplement que le premier ait pris et libéré les deux verrous.
* **La solution :** Remplacer les deux `lock_guard` successifs par un unique **`std::scoped_lock lock(s1.mtx, s2.mtx);`** (C++17), qui verrouille plusieurs mutex simultanément grâce à un algorithme interne anti-deadlock, quel que soit l'ordre des paramètres.

### 5. Verrouiller deux fois le même mutex dans le même thread *(QCM 3)*

* **Le piège :** Appeler `fuse(imu, imu)` en passant deux fois le même objet à une fonction qui verrouille `s1.mtx` puis `s2.mtx`.
* **Pourquoi c'est un bug :** Un `std::mutex` standard n'est **pas récursif**. Lorsqu'un thread tente de verrouiller une seconde fois un mutex qu'il possède déjà, il se met en pause en attendant qu'il se libère lui-même (**auto-deadlock**).
* **La solution :** Vérifier que les deux objets sont distincts (`if (&s1 == &s2)`) avant de verrouiller, ou utiliser `std::recursive_mutex` si le même thread doit pouvoir acquérir plusieurs fois le même verrou.

### 6. Opérateur `++` et booléens partagés sans protection *(QCM 3 et 5)*

* **Le piège :** Modifier un compteur global `fusion_count++` ou lire une condition d'arrêt `while (running)` depuis plusieurs threads sur des types primitifs standards (`int`, `bool`) sans mutex.
* **Pourquoi c'est un bug :** C'est une **Data Race** (comportement indéfini). Au niveau du processeur, `count++` se décompose en trois étapes (lecture en mémoire, addition dans un registre, écriture en mémoire) qui peuvent s'entremêler entre deux threads. Pour `while (running)`, le compilateur peut optimiser le code en gardant la valeur `true` en cache dans un registre du CPU, empêchant la boucle de voir que l'autre thread a passé `running` à `false`.
* **La solution :** Déclarer ces variables avec **`std::atomic<int>`** et **`std::atomic<bool>`**, ce qui garantit des opérations indivisibles et synchronisées au niveau matériel sans la lourdeur d'un mutex.

### 7. Oubli de `join()` ou `detach()` avant la fin de vie d'un thread *(QCM 5)*

* **Le piège :** Instancier un `std::thread t(...)` dans une fonction et arriver à l'accolade fermante `}` sans avoir appelé `t.join()` ni `t.detach()`.
* **Pourquoi c'est un bug :** Cela provoque un **crash immédiat** du programme. Lorsque la variable locale `t` est détruite à la fin de la fonction, son destructeur `~thread()` vérifie si le thread est toujours actif (*joinable*). Si c'est le cas, la norme C++ l'oblige à appeler `std::terminate()`, ce qui tue brutalement le processus.
* **La solution :** Toujours appeler **`t.join()`** avant la fin de la portée de `t` (ou utiliser `std::jthread` en C++20 qui effectue le `join()` automatiquement dans son destructeur).

### 8. Le danger de `detach()` sur des variables locales et l'usage de `std::ref` *(QCM 5)*

* **Le piège :** Passer une variable locale `data` par référence à un thread avec `std::ref(data)`, puis appeler `t.detach()` avant de quitter la fonction.
* **Pourquoi c'est un bug :**
1. D'abord, `std::thread` copie par défaut tous ses arguments par valeur ; sans **`std::ref(data)`**, le code ne compile pas si la fonction attend une référence modifiable `std::vector<double>&`.
2. Ensuite, appeler `t.detach()` permet à la fonction principale de se terminer immédiatement pendant que le thread continue de tourner en arrière-plan. Dès que la fonction se termine, ses variables locales (`worker` et `data`) sont détruites de la pile (*stack*). Le thread détaché continue alors d'écrire dans une zone mémoire détruite (**Use-After-Free**).


* **La solution :** Ne jamais utiliser `detach()` si le thread manipule des variables locales. Appeler **`t.join()`** pour bloquer la sortie de la fonction tant que le thread n'a pas terminé son travail.

---

## Partie 2 : Les 7 Pièges de la POO & Gestion Mémoire (QCM 2, 4 et 6)

### 9. Object Slicing (Troncature d'objet) *(QCM 2)*

* **Le piège :** Passer un objet dérivé (`KalmanFilter`) à une fonction qui prend la classe mère **par valeur** : `void runCycle(StateEstimator estimator)`.
* **Pourquoi c'est un bug :** Le passage par valeur crée une copie de l'objet sous la forme d'un simple `StateEstimator`. Toute la partie spécifique à `KalmanFilter` (ses attributs et sa table virtuelle) est littéralement coupée (*sliced*). L'appel `estimator.predict()` exécutera la méthode de la classe mère au lieu de celle de la classe fille.
* **La solution :** Toujours passer les objets polymorphes **par référence** (`const StateEstimator&` ou `StateEstimator&`) ou par pointeur pour éviter la copie et conserver le comportement dynamique de la classe fille.

### 10. Absence de destructeur `virtual` dans la classe mère *(QCM 2 et 4)*

* **Le piège :** Manipuler un objet enfant via un pointeur de la classe mère (`StateEstimator* ptr = new KalmanFilter();`) et faire `delete ptr;` alors que le destructeur `~StateEstimator()` n'est pas déclaré `virtual`.
* **Pourquoi c'est un bug :** Cela provoque une **fuite mémoire** et un comportement indéfini. Sans le mot-clé `virtual`, le compilateur se fie au type statique du pointeur (`StateEstimator*`) et n'appelle que le destructeur de la classe mère. Le destructeur `~KalmanFilter()` n'est jamais appelé, et aucun de ses attributs (qu'il s'agisse d'un pointeur `new[]`, d'un `std::vector` ou d'un `std::shared_ptr`) n'est libéré.
* **La solution :** Dès qu'une classe est destinée à être héritée ou possède au moins une méthode virtuelle, déclarer systématiquement son destructeur comme virtuel : **`virtual ~Base() = default;`**.

### 11. Appel d'une méthode virtuelle dans un constructeur *(QCM 4)*

* **Le piège :** Appeler une méthode virtuelle `init()` à l'intérieur du constructeur de la classe mère `Controller()`, et s'attendre à ce que cela exécute la version redéfinie `AttitudeController::init()` lorsqu'on instancie un objet enfant.
* **Pourquoi c'est un bug :** En C++, la classe mère est toujours construite **avant** la classe fille. Pendant que le constructeur `Controller()` s'exécute, les attributs de `AttitudeController` ne sont pas encore initialisés en mémoire. Pour empêcher que la méthode fille n'accède à des variables non initialisées, le C++ désactive la descente polymorphe dans les constructeurs : l'objet se comporte comme une simple classe mère et affiche `"Base init"`.
* **La solution :** Ne jamais appeler de fonction virtuelle dans un constructeur ou un destructeur. Appeler `init()` dans une étape séparée une fois l'objet complètement construit.

### 12. Copie interdite de `std::unique_ptr` *(QCM 4)*

* **Le piège :** Écrire `std::unique_ptr<Controller> p2 = p1;` pour affecter un pointeur unique à un autre.
* **Pourquoi c'est un bug :** Cela provoque une **erreur de compilation**. Le rôle de `std::unique_ptr` est de garantir qu'une ressource mémoire possède un seul et unique propriétaire à tout moment. Son constructeur de copie et son opérateur d'affectation par copie sont explicitement désactivés (`= delete`).
* **La solution :** Transférer la propriété avec **`std::move(p1)`** (ce qui vide `p1` au profit de `p2`), ou utiliser `std::shared_ptr` si plusieurs objets doivent réellement partager la propriété de la ressource.

### 13. Références circulaires avec `std::shared_ptr` *(QCM 4)*

* **Le piège :** Un objet A possède un `std::shared_ptr` vers un objet B, et l'objet B possède en retour un `std::shared_ptr` vers l'objet A.
* **Pourquoi c'est un bug :** Cela crée une **fuite mémoire**. Un `shared_ptr` ne détruit l'objet pointé que lorsque son compteur de références tombe à zéro. Ici, même lorsque le programme n'utilise plus A ni B, A maintient le compteur de B à 1 et B maintient le compteur de A à 1. Ils ne sont jamais détruits.
* **La solution :** Casser le cycle en remplaçant l'un des deux pointeurs par un **`std::weak_ptr`**, qui permet d'observer et d'accéder à l'objet sans incrémenter son compteur de références.

### 14. Copie superficielle (*Shallow Copy*) d'un pointeur brut *(QCM 6)*

* **Le piège :** Copier un objet (`RadarSensor r2 = r1;`) d'une classe qui alloue de la mémoire avec `new[]` dans un pointeur brut `double* history`, mais qui ne définit pas de constructeur de copie personnalisé.
* **Pourquoi c'est un bug :** Le compilateur génère un constructeur de copie par défaut qui effectue une **copie superficielle (*shallow copy*)** : il copie uniquement l'adresse mémoire stockée dans le pointeur. Les objets `r1` et `r2` pointent alors vers le même tableau en mémoire. À la fin de la fonction, le destructeur de `r2` libère le tableau avec `delete[]`, puis le destructeur de `r1` tente de libérer une seconde fois la même adresse (**Double Free / Crash**).
* **La solution :** Appliquer la **Règle de Zéro** en remplaçant `double*` par `std::vector<double>` (qui gère automatiquement sa propre copie profonde), ou appliquer la **Règle des Trois** en écrivant un constructeur de copie et un opérateur `=` qui allouent un nouveau tableau indépendant (*deep copy*).

### 15. Classe abstraite et méthodes virtuelles pures (`= 0`) *(QCM 6)*

* **Le piège :** Croire que le compilateur refusera de compiler une classe fille `LaserSensor` qui hérite d'une interface ayant `virtual double readAltitude() = 0;` sans implémenter cette méthode, même si on n'instancie jamais `LaserSensor`.
* **Pourquoi c'est un bug de raisonnement :** Une méthode déclarée avec `= 0` est une **méthode virtuelle pure**, ce qui rend la classe **abstraite**. Si une classe fille ne rédige pas cette méthode, elle hérite simplement de son caractère abstrait : cela est parfaitement légal en C++ et compile sans aucune erreur. L'erreur de compilation n'apparaît que si l'on tente d'instancier concrètement un objet de cette classe (`ISensor s;` ou `LaserSensor ls;`).
* **La solution :** Implémenter toutes les méthodes virtuelles pures (`= 0`) avec le mot-clé `override` uniquement dans les classes finales concrètes que l'on souhaite instancier.

---

## Les 4 réflexes rapides pour les QCM le jour du test

1. **Présence de `new` / `delete` ou d'un pointeur brut en attribut :** Vérifie immédiatement si le destructeur de la classe mère est bien `virtual` et si un objet est copié par valeur (`r2 = r1`, ce qui cause un *double free*).
2. **Paramètre passé par valeur `void f(Base b)` :** C'est systématiquement un piège d'*Object Slicing* ; le polymorphisme est perdu et c'est la méthode de la classe mère qui s'exécutera.
3. **Verrouillage manuel `mtx.lock()` ou double `lock_guard` :** Cherche une exception qui empêcherait d'atteindre `unlock()`, ou une inversion de l'ordre des arguments entre deux threads qui provoquerait un *deadlock*.
4. **Création d'un `std::thread` :** Vérifie qu'il y a bien `.join()` avant l'accolade fermante, que les références passées utilisent `std::ref()`, et qu'aucun `detach()` ne laisse tourner un thread sur des variables locales.