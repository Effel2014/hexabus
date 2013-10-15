var UpdateControl = function(element, config) {
	config = config || {};

	var cssApply = function(key, control) {
		if (key in config)
			control.css(key, config[key]);
	};

	var container = $('<div style="position: absolute" />');
	cssApply("left", container);
	cssApply("top", container);

	var box = $('<input type="text" style="position: relative" />');
	cssApply("width", box);
	cssApply("height", box);
	box.attr("value", config.value);
	container.append(box);

	var button = $('<input type="button" value="OK" style="position: absolute" />')
		.css("left", box.css("right"));
	container.append(button);

	if (config.font != undefined) {
		for (key in config.font) {
			box.css(key, config.font[key]);
			button.css(key, config.font[key]);
		}
	}



	var finish = function(orderly) {
		container.remove();
		if (!orderly && config.abort) {
			config.abort();
		}
	};

	var accept = function() {
		finish(true);
		if (config.done) {
			config.done(box.prop("value"));
		}
	};

	var focusLostHandler;
	var cancelFocusLost = function() {
		if (focusLostHandler != undefined) {
			window.clearTimeout(focusLostHandler);
		}
	};
	var focusLost = function() {
		cancelFocusLost();
		focusLostHandler = window.setTimeout(finish, 200);
	};

	var focusedControl = box;
	this.focus = function() {
		focusedControl.focus();
	};

	box.blur(focusLost);
	box.focus(function() {
		cancelFocusLost();
		focusedControl = box;
	});
	box.keydown(function(ev) {
		if (ev.which == 13) {
			accept();
			ev.preventDefault();
		} else if (ev.which == 27) {
			finish(false);
		}
	});

	button.blur(focusLost);
	button.focus(function() {
		cancelFocusLost();
		focusedControl = button;
	});
	button.click(accept);

	element.append(container);
	this.focus();
};

angular.module('controls', [
])
.directive('visGauge', [function() {
	return {
		restrict: 'A',
		scope: {
			endpoint: '=',
			control: '=',
			enableEdit: '@',
			hasValue: '=',
			editBegin: '=',
			editDone: '=',
			id: '@',
			cssClass: '@'
		},
		replace: false,
		template:
			'<div data-ng-gauge="gauge" ' + 
				'id="{{endpoint.id}}" ' +
				'class="{{cssClass}} {{cssClass}}-{{endpoint.ip}}" ' +
				'data-title-text="{{endpoint.name}} [{{endpoint.unit}}]" ' +
				'data-min="{{endpoint.minvalue}}" ' +
				'data-max="{{endpoint.maxvalue}}" ' +
				'data-value="{{endpoint.value}}" ' +
				'data-gauge="control.gauge" ' +
				'data-disabled="!hasValue" ' +
				'data-min-click="minClick()" ' +
				'data-max-click="maxClick()" ' +
				'data-title-click="titleClick()">' +
			'</div>',
		compile: function(element, attrs, transclude) {
			return {
				post: function(scope, element, attrs, controller) {
					scope.control = {};
					var pendingUpdateControl = null;

					var placeUpdateControl = function(control, attrs, done) {
						var element = $(document.getElementById(scope.endpoint.id));
						var bound = control.getBBox();

						attrs = attrs || {};

						var config = {
							left: attrs.left || (bound.x + "px"),
							top: attrs.top || (bound.y + "px"),
							width: attrs.width || ((bound.x2 - bound.x) + "px"),
							height: attrs.height || ((bound.y2 - bound.y) + "px"),

							done: done,
							font: {},

							value: attrs.text || control.attrs.text
						};
						for (key in control.attrs) {
							if (typeof key == "string" && key.substr(0, 4) == "font") {
								config.font[key] = control.attrs[key];
							}
						}

						pendingUpdateControl = new UpdateControl(element, config);
					};

					scope.control.cover = function() {
						var div = $('<div class="spinner-large transient"></div>' +
							'<div class="updating-thus-disabled transient"></div>');
						$(document.getElementById(scope.endpoint.id))
							.append(div);
					};

					scope.control.coverClass = function() {
						var div = $('<div class="spinner-large transient"></div>' +
							'<div class="updating-thus-disabled transient"></div>');
						$(document.getElementsByClassName(scope.cssClass + "-" + scope.endpoint.ip))
							.append(div);
					};

					scope.control.uncover = function() {
						$(document.getElementById(scope.endpoint.id))
							.children(".transient").remove();
					};

					scope.control.uncoverClass = function() {
						$(document.getElementsByClassName(scope.cssClass + "-" + scope.endpoint.ip))
							.children(".transient").remove();
					};

					scope.minClick = function() {
						if (!scope.enableEdit) {
							return;
						}
						scope.editBegin(scope.endpoint, { target: "minvalue" });
						placeUpdateControl(
								scope.control.gauge.txtMin,
								null,
								function(value) {
									scope.editDone(scope.endpoint, {
										minvalue: +value
									});
									pendingUpdateControl = null;
								});
					};

					scope.maxClick = function() {
						if (!scope.enableEdit) {
							return;
						}
						scope.editBegin(scope.endpoint, { target: "maxvalue" });
						placeUpdateControl(
								scope.control.gauge.txtMax,
								null,
								function(value) {
									scope.editDone(scope.endpoint, {
										maxvalue: +value
									});
									pendingUpdateControl = null;
								});
					};

					scope.titleClick = function() {
						if (!scope.enableEdit) {
							return;
						}
						scope.editBegin(scope.endpoint, { target: "name" });
						placeUpdateControl(
								scope.control.gauge.txtTitle,
								{
									left: "0px",
									width: "140px",
									text: scope.endpoint.name
								},
								function(value) {
									scope.editDone(scope.endpoint, {
										name: value
									});
									pendingUpdateControl = null;
								});
					};

					scope.control.focus = function() {
						if (pendingUpdateControl) {
							pendingUpdateControl.focus();
						}
					};
				}
			};
		}
	};
}])
.directive('visEndpoint', [function() {
	return {
		restrict: 'A',
		scope: {
			num: '@',
			endpoint: '=',
			enableEdit: '@',
			editBegin: '=',
			editDone: '=',
		},
		replace: true,
		template:
			'<div data-ng-switch="endpoint.ep_desc.function">' + 
				'<div data-ng-switch-when="sensor">' +
					'<div data-vis-gauge="" ' + 
						'data-css-class="gauge" ' +
						'data-endpoint="endpoint.ep_desc" ' +
						'data-enable-edit="{{enableEdit}}" ' +
						'data-edit-begin="editBegin" ' +
						'data-edit-done="editDone" ' +
						'data-has-value="endpoint.ep_desc.has_value" ' +
						'data-control="endpoint.control">' + 
					'</div>' +
				'</div>' + 
			'</div>',
		compile: function(element, attrs, transclude) {
			return {
				post: function(scope, element, attrs, controller) {
				}
			};
		}
	};
}]);
