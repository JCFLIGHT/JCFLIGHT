# Desenvolvimento

Documento apenas para desenvolvedores.

## Observações Gerais

1. O software é de origem brasileira,mas é aceito apenas o desenvolvimento em inglês,não escreva nada
em português,em excessão os comentários,a nossa lingua não é bonita para escrever algoritimos.
2. As funções de retorno `bool` devem ser nomeadas inicialmente como 'Get',por exemplo,
'GetGroundDetected'.
3. Evite palavras-ruido em nomes de variaveis,pense no que você está nomeando e nomeie bem.
4. Evite comentários que descrevam o que o código está fazendo,o código deve se descrever.Os comentários são úteis, entretanto,para fins de visão geral e para documentar o conteúdo das variáveis.
5. Os comentários devem ser sempre escritos em maiusculos,por mais que pareça que você está gritando (Caps Lock),mas é assim que tem que ser.
6. Se você precisar documentar uma variável,faça na declaração da mesma,não copie o comentário para o `extern` uso, pois isso levará à podridão do comentário.
7. Procure conselhos de outros desenvolvedores,saiba que você sempre pode aprender mais.
8. Saiba que sempre há mais de uma maneira de fazer algo e que o código nunca é final,o que importa é fazer funcionar.

## Estilo de Codificação

* Sempre coloque a primeira letra de uma palavra em maiuscula,por exemplo:

```
uint8_t Counter = 0;

void LoopFunctionTest()
{
}
```

* Se o seu código for semelhante a

```
for(i=0;i<10;i++)test(i);
```

* ele será excluído imediatamente.

* Coisas como

```
if (Foo) Bar(); 
```
* não será aceito.Colocar tudo em uma única linha não o torna mais rápido.

* As condições if/else que contém apenas uma única linha devem usar chaves de abertura e fechamento:

```
if (Foo)
{
 Bar();
}
else
{
 Test();
} 
```

* A orientação acima também se aplica a funções for/while:

```
for (uint8_t IndexCount = 0; IndexCount < 10; IndexCount++)
{
 DEBUG("Test\n");
}

while (IndexCount--)
{
 Blink();
}

while (true) 
{
 Loop();
 Test();
}
```

* Não utilize 'i' para nomear a incrementação ou decrementação de um laço for(),use:

```
for (uint8_t IndexCount = 0; IndexCount < 10; IndexCount++)
{
}
```

* Comentários embutidos devem usar a notação de comentário C++ "//".

```
//COMENTARIO TESTE
```

* Caso o argumento de uma função não for utilizado,não deixe como:

```
void Foo();
```

estará correto assim:

```
void Foo(void);
```

* As estruturas devem conter '_Stuct' no final do nome e,as variaveis dentro deverão ter um valor inicial,por exemplo:

```
typedef struct
{
 uint8_t VarTest = 0;
}Test_Struct;
```
## Variáveis

Não será aceito declarações de variaveis usando:

```
byte
short
unsigned short
int
unsigned int
```

estará correto assim:

```
int8_t
uint8_t
int16_t
uint16_t
int32_t
uint32_t
```

## Nomeclatura de arquivo

Os nomes das pastas devem ser escritas em minusculo,exemplo:

```
Barometer
Barometer
```

Os nomes dos arquivos devem ser escritos em maiusculos,exemplo:

```
BAROMETER.cpp
BAROMETER.h
```

## Qualidade do código

Código ruim não será aceito,tem que funcionar bem e,ter uma boa aparência.