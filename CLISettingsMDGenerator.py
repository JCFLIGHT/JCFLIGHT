import yaml
import codecs
import pathlib

SETTINGS_YAML_PATH = "Settings.yaml"


def Parse_Settings_YAML():
    with codecs.open(SETTINGS_YAML_PATH, "r", "utf-8-sig") as Settings_YAML:
        return yaml.load(Settings_YAML, Loader=yaml.Loader)


def Generate_MD_Doc_From_YAML(Settings_YAML):
    Params = {}

    for Member in Settings_YAML['Members']:

        Params[Member['Name']] = {
            "Descrição": Member["Description"] if "Description" in Member else "",
            "Valor Padrão": Member["DefaultValue"] if "DefaultValue" in Member else "",
            "Min": Member["Min"] if "Min" in Member else "",
            "Max": Member["Max"] if "Max" in Member else ""
        }

    MD_Table = [
        "| Nome | Valor Padrão | Min | Max | Descrição |\n",
        "| ------------- | ------------- | --- | --- | ----------- |\n",
    ]

    for Param in sorted(Params.items()):
        MD_Table.append("| {} | {} | {} | {} | {} |\n".format(
            Param[0], Param[1]['Valor Padrão'], Param[1]['Min'], Param[1]['Max'], Param[1]['Descrição']
        ))

    return MD_Table


try:
    print('')

except:
    print('')

with codecs.open(pathlib.PurePath('__main__').parent / "Settings.md", "w", "utf-8-sig") as Settings_md:
    Settings_YAML = Parse_Settings_YAML()
    MD_Table = Generate_MD_Doc_From_YAML(Settings_YAML)
    Settings_md.writelines(["# Variáveis do CLI\n", "\n"] + MD_Table + ["\n",
                                                                        "> Esse arquivo é gerado automaticamente,não o edite manualmente!"])
