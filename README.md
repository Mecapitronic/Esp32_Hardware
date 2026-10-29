# Esp32_Hardware
Lib for common hardware mounted on PAMI and main robot

## Commandes console

Envoyer `Help` pour voir les commandes des modules initialisés et la légende.
Les aides suivent la [convention ESP32_Helper](../Esp32_Helper/README.md#aide-et-syntaxe-des-commandes) :

- `<nom:type@unite>` : paramètre obligatoire.
- `(:nom:type@unite=defaut)` : paramètre facultatif, séparateur compris.
- `int` : entier signé 32 bits ; `bool` : `0` ou `1` ; `string` : texte.
- L'unité et la valeur par défaut sont omises si elles ne s'appliquent pas.
  `deg` signifie degrés, `%` pourcentage, `ms` millisecondes et `bit/s` débit série.

Ne pas saisir les annotations : `AX12Torque:<id:int>:<torque:int@%>` se transmet
par exemple sous la forme `AX12Torque:1:30`, terminée par newline ou `;`.
Les contraintes de l'aide ne constituent pas une validation automatique.

### Match

```text
MatchNumPami:<number:int> - Set the PAMI number; number: 1..99.
```

### Servos AX12

Le paramètre `id` désigne toujours un **identifiant logique enregistré**, pas
l'adresse physique Dynamixel sur le bus. `AX12ConfigPrint` permet de consulter
la configuration des servos.

```text
AX12Scan - Scan all supported protocols and baud rates.
AX12Scan:<protocol:int>:<baudRate:int@bit/s> - Scan one protocol and baud rate.
AX12PrintInfo(:id:int) - Show information for one servo, or all if omitted.
AX12ConfigPrint - Show all registered servo configurations.
AX12Start - Enable torque on all registered servos.
AX12Stop - Disable torque on all registered servos.
AX12Wait(:id:int) - Wait for one servo, or all if omitted.

AX12Pos - Show all servo positions.
AX12Pos:<id:int> - Show one servo position.
AX12Pos:<id:int>:<position:int@deg>(:timeout:int@ms=10000) - Show position, then set the target.

AX12Speed - Show all servo speeds.
AX12Speed:<id:int> - Show one servo speed.
AX12Speed:<id:int>:<speed:int@%> - Show speed, then set it.

AX12Torque - Show all raw servo torque limits (0..1023).
AX12Torque:<id:int> - Show one raw servo torque limit (0..1023).
AX12Torque:<id:int>:<torque:int@%> - Show raw torque limit, then set it in percent.

AX12Table:<item:int> - Read a control-table item for all servos.
AX12Table:<item:int>:<value:int>(:id:int) - Read all, then write to one servo or all if omitted.
```

- `AX12Scan` : `protocol` vaut `1` ou `2`. Le balayage sans arguments teste les
  débits 57600, 115200, 1000000, 2000000 et 3000000 bit/s.
- `AX12Pos` : la position doit rester dans les limites configurées du servo ;
  le timeout de mouvement est exprimé en ms, attendu strictement positif,
  et vaut 10000 ms s'il est omis. Les positions affichées sont en degrés.
- `AX12Speed` et `AX12Torque` : valeurs attendues de 0 à 100 %.
  Les limites affichées par `AX12Torque` restent les valeurs brutes du registre
  (0 à 1023), alors que le paramètre de réglage est en pourcentage.
- `AX12Table` : `item` est l'index d'un élément de la table
  `Dynamixel2Arduino`, **pas l'adresse d'un registre**. La valeur utilise les
  unités brutes du registre, pas nécessairement les degrés ou pourcentages des
  commandes de haut niveau. La lecture porte toujours sur tous les servos,
  même lorsqu'un `id` limite ensuite l'écriture.

Exemple à saisir :

```text
AX12Torque:1:30;AX12Speed:1:50;AX12Pos:1:150:2000;AX12Wait:1;
```

Adapter l'identifiant et la position à la configuration du robot.
