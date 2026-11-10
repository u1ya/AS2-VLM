"""Conservative bilingual command vocabulary; quantities here use metres/degrees.

Only complete clauses match. Unknown text is never discarded or fuzzy-matched.
The model uses the same semantic intents for other paraphrases.
"""
import math
import re


VOCABULARY = """Command meanings (Russian and English):
look_around: оглянись вокруг / осмотрись / look around = whole robot turns 360 degrees on the spot.
turn_around: развернись / turn around = whole robot turns 180 degrees.
turn_left / turn_right without an angle = 90 degrees.
For look_around and turn_around, default direction is LEFT; explicit right overrides it.
walk_forward / walk_backward always need a distance. немного / a little needs clarification.
Preserve explicit numbers, units, direction and order. Never infer a distance to an object.
Negated, quoted, conditional or hypothetical commands are not immediate movement orders.
"""

SPLIT = re.compile(r"\s*(?:;|,(?!\d)|(?<!\d)\.(?!\d)|\band then\b|\bafter that\b|\bthen\b|\band\b|\bпосле этого\b|\bзатем\b|\bпотом\b|\bи\b)\s*")
LEFT = r"left|counterclockwise|counter-clockwise|налево|влево|против часовой стрелки"
RIGHT = r"right|(?<!counter-)clockwise|направо|вправо|по часовой стрелке"
DIRECTION = rf"(?:{LEFT}|{RIGHT})"
NUMBERS = {"one": "1", "two": "2", "half a": "0.5", "half": "0.5", "a quarter": "90",
           "quarter": "90", "полметра": "0.5 метра", "полтора": "1.5",
           "один": "1", "два": "2", "пятьдесят": "50", "fifty": "50"}


def normalize(text):
    return re.sub(r"\s+", " ", text.lower().replace("ё", "е").replace("’", "'")).strip().strip(".!? ")


def clauses(text):
    text = normalize(text)
    text = re.sub(r"^(?:please|пожалуйста)[, ]+", "", text)
    text = re.sub(r"[, ]+(?:please|пожалуйста)$", "", text)
    return [normalize(part) for part in SPLIT.split(text) if part.strip()]


def needs_literal_clarification(text):
    """Do not let a model turn a prohibition or a discussion into an actuator plan."""
    return bool(re.search(
        r"\b(?:не|нельзя|если|когда|скажи|произнеси|объясни|don't|dont|do not|never|not|if|when|say|explain)\b|[\"«»“”]",
        normalize(text)))


def ambiguous_motion(text):
    for part in clauses(text):
        if re.search(r"\b(?:немного|чуть|a little|a bit|somewhat)\b", part):
            return True
        if re.fullmatch(r"(?:go|walk|move) (?:forward|backward|back)|(?:иди|пройди) (?:вперед|назад)|(?:повернись|turn|rotate)", part):
            return True
        if re.search(rf"\b(?:{LEFT})\b", part) and re.search(rf"\b(?:{RIGHT})\b", part):
            return True
    return False


def turn_constraint(text):
    """Independent semantic checks for turns embedded in unfamiliar paraphrases.

    This does not authorize execution: it can only reject an inconsistent model answer.
    """
    text = normalize(text)
    angle = None
    if re.search(r"\b(?:полный (?:оборот|поворот)|full turn|full rotation|all the way round)\b", text):
        angle = 360
    elif re.search(r"\b(?:в обратную сторону|opposite direction|half turn|half rotation)\b", text):
        angle = 180
    elif re.search(r"\b(?:четверть оборота|quarter turn)\b", text):
        angle = 90
    right = bool(re.search(rf"\b(?:{RIGHT}|правый бок)\b", text))
    left = bool(re.search(rf"\b(?:{LEFT}|левый бок)\b", text))
    return angle, "turn_right" if right and not left else "turn_left" if (left and not right) or angle is not None else None


def target_navigation(text):
    return any(re.match(
        r"^(?:please )?(?:(?:go|walk|move|head) (?:to|towards)\b|approach\b)|"
        r"^(?:пожалуйста[, ]+)?(?:дойди|подойди|приблизься)\b", part)
        for part in clauses(text))


def _step(tool, value):
    return {"tool": tool, "value": float(value), "reasoning": "Recognized complete operator command"}


def parse_clause(text):
    """Return a semantic action or None; never extract an action from partial text."""
    text = normalize(text)
    text = re.sub(r"^(?:please|пожалуйста)[, ]+", "", text)
    text = re.sub(r"[, ]+(?:please|пожалуйста)$", "", text)
    if text in {"stop", "stop moving", "halt", "стоп", "остановись", "останови движение"}:
        return _step("stop", 0)
    # Default direction applies only to the named, fixed-angle intents.
    for intent, pattern, amount in (
        ("look_around", r"оглянись(?: вокруг)?|осмотрись(?: вокруг)?|посмотри вокруг|look around|spin around|make a full turn|сделай полный (?:оборот|поворот)(?: на месте)?", 360),
        ("turn_around", r"развернись|повернись кругом|turn around|make a half turn", 180),
    ):
        match = re.fullmatch(rf"(?:{pattern})(?: (?:to (?:the )?)?({DIRECTION}))?", text)
        if match:
            return _step("turn_right" if match[1] and re.fullmatch(RIGHT, match[1]) else intent, amount)
    match = re.fullmatch(r"повернись в обратную сторону(?: через (левый|правый) бок)?", text)
    if match:
        return _step("turn_right" if match[1] == "правый" else "turn_around", 180)
    match = re.fullmatch(r"face the opposite direction(?: with a (clockwise|counterclockwise|counter-clockwise) half turn)?", text)
    if match:
        return _step("turn_right" if match[1] == "clockwise" else "turn_around", 180)
    match = re.fullmatch(rf"(?:turn|повернись|поверни) (?:to (?:the )?)?({DIRECTION})", text)
    if match:
        return _step("turn_right" if re.fullmatch(RIGHT, match[1]) else "turn_left", 90)
    # Number words are deliberately bounded; typo guessing belongs to clarification.
    for word, number in NUMBERS.items():
        text = re.sub(rf"\b{re.escape(word)}\b", number, text)
    text = re.sub(r"\b(?:a|an) (met(?:er|re))\b", r"1 \1", text)
    text = re.sub(r"\b(на|вперед|назад) метр$", r"\1 1 метр", text)
    text = re.sub(r"^(пройди|пройдите|walk) (?=\d)", r"\1 вперед " if text.startswith("прой") else r"\1 forward ", text)
    match = re.fullmatch(
        r"(?:go|walk|move|пройди|иди|пройдите) (forward|forwards|back|backward|backwards|вперед|назад) "
        r"(?:(?:for|by|на) )?(\d+(?:[.,]\d+)?) (meters?|metres?|m|метр[аов]*|centimeters?|centimetres?|cm|сантиметр[аов]*|см)", text)
    if not match:
        match = re.fullmatch(r"(back up|отступи|отойди назад|сдай назад) (?:на )?(\d+(?:[.,]\d+)?) (meters?|metres?|m|метр[аов]*|centimeters?|centimetres?|cm|сантиметр[аов]*|см)", text)
    if match:
        value = float(match[2].replace(",", "."))
        if match[3].startswith(("cent", "сант")) or match[3] in {"cm", "см"}:
            value /= 100
        return _step("walk_backward" if match[1] in {"back", "backward", "backwards", "назад", "back up", "отступи", "отойди назад", "сдай назад"}
                     else "walk_forward", value)
    match = re.fullmatch(
        rf"(?:rotate|turn|повернись|поверни) (?:(?:to (?:the )?)?({DIRECTION}) )?"
        rf"(?:(?:by|на) )?(\d+(?:[.,]\d+)?) ?(degrees?|deg|°|градус[аов]*|radians?|rad)"
        rf"(?: (?:to (?:the )?)?({DIRECTION}))?", text)
    if match:
        before, amount, unit, after = match.groups()
        if before and after and bool(re.fullmatch(RIGHT, before)) != bool(re.fullmatch(RIGHT, after)):
            return None
        # Radians are converted by code, never by the model.
        value = float(amount.replace(",", "."))
        if unit in {"radian", "radians", "rad"}:
            value = math.degrees(value)
        return _step("turn_right" if re.fullmatch(RIGHT, before or after or "left") else "turn_left", value)
    return None
